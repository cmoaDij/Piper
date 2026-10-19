"""Local browser-based multi-view review, with explicit per-episode decisions."""

import json
import secrets
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from data_collect.common.config import IMAGE_STREAMS, atomic_json, fingerprint
from data_collect.recording.images import EpisodeImages, validate_images

HTML = """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>Piper 数据质检</title><style>
body{font:16px system-ui;background:#101820;color:#e6edf5;margin:24px}button,select,input{font:inherit;padding:8px;margin:6px}
#views{display:grid;grid-template-columns:1fr 1fr;gap:8px;max-width:1000px}img{width:100%;height:180px;object-fit:contain;background:#17232f}pre{white-space:pre-wrap;font-size:13px}
h1{font-size:26px;margin-bottom:10px}details{margin-top:10px}.pass{color:#8fe3a0}.reject{color:#ffadad}#seek{width:70%}
#empty{white-space:pre-line;color:#ffd68a}#views[hidden]{display:none}button:disabled{opacity:.45;cursor:not-allowed}</style>
<h1>Episode 人工质检</h1><p>核对四路图像及夹爪动作；自动 REJECT 无法人工放行。待审核数据不会进入转换白名单。</p>
<button id="refresh">刷新列表</button><p id="empty" role="status"></p>
<select id="episodes"></select><button id="play">播放 / 暂停</button>
<input id="seek" type="range" value="0" min="0" max="0"><span id="position"></span>
<div id="views"></div>
<input id="operator" value="operator_01" aria-label="审核人"><input id="note" placeholder="筛选备注">
<button id="approve">通过此条</button><button id="reject">剔除此条</button><button id="pending">恢复待审核</button>
<pre id="result"></pre><details><summary>自动质检详情</summary><pre id="report"></pre></details>
<details><summary>当前关节状态与指令</summary><pre id="state"></pre></details><script>
const token='__TOKEN__', streams=['cameras/global_rgb','cameras/wrist_rgb','tactile/left_raw','tactile/right_raw'];
const $=id=>document.getElementById(id); let rows=[], playing=false, busy=false, pendingFrame=false;
for(const s of streams){const box=document.createElement('div'),label=document.createElement('div'),img=document.createElement('img');
label.textContent=s;img.id=s;img.alt=s;box.append(label,img);$('views').append(box);}
function current(){return rows[Number($('episodes').value)]}
async function frame(){if(!rows.length)return;$('approve').disabled=true;if(busy){pendingFrame=true;return}busy=true;
const row=current(), k=Number($('seek').value);$('position').textContent=`${k+1}/${row.frames}`;
const query=`episode=${encodeURIComponent(row.episode)}&k=${k}`;
let loaded=false;try{const results=await Promise.all(streams.map(s=>new Promise(resolve=>{const img=$(s);img.onload=()=>resolve(true);img.onerror=()=>resolve(false);img.src='/api/frame?'+query+'&stream='+encodeURIComponent(s)})));
loaded=results.every(Boolean);const r=await fetch('/api/state?'+query);const state=await r.json();if(row===current())$('state').textContent=JSON.stringify(state,null,2);
}catch(error){$('result').textContent=String(error)}finally{busy=false;if(pendingFrame){pendingFrame=false;frame()}else $('approve').disabled=!loaded||!current()||current().status!=='PASS';}}
function select(){playing=false;const row=current();if(!row)return;$('seek').max=row.frames-1;$('seek').value=0;
$('report').textContent=JSON.stringify(row,null,2);$('approve').disabled=row.status!=='PASS';frame();}
function emptyState(message){playing=false;rows=[];$('episodes').replaceChildren();$('views').hidden=true;
for(const id of ['episodes','play','seek','approve','reject','pending'])$(id).disabled=true;
$('empty').textContent=message;$('position').textContent='';$('report').textContent='';$('state').textContent='';}
async function refresh(){playing=false;try{const response=await fetch('/api/episodes');const data=await response.json();
if(!response.ok)throw new Error(data.error||'审核列表读取失败');rows=data;$('episodes').replaceChildren();
if(!rows.length){emptyState('暂无可审核的轨迹。完成采集后，在 Data_Collect 目录重新运行：\\npython3 -m data_collect.cli.review\\n程序会自动质检并加载审核列表。');return}
$('empty').textContent='';$('views').hidden=false;for(const id of ['episodes','play','seek','reject','pending'])$(id).disabled=false;
rows.forEach((r,i)=>{let o=document.createElement('option');o.value=i;o.textContent=`${r.episode} | ${r.status} | ${r.review}`;$('episodes').append(o)});select();
}catch(error){emptyState('无法读取审核列表：'+error.message+'\\n处理后点击“刷新列表”重试。')}}
$('refresh').onclick=refresh;
$('episodes').onchange=select;$('seek').oninput=frame;$('play').onclick=()=>playing=!playing;
setInterval(()=>{if(playing&&!busy){if(Number($('seek').value)<Number($('seek').max)){$('seek').value=Number($('seek').value)+1;frame()}else playing=false}},50);
for(const decision of ['approve','reject','pending'])$(decision).onclick=async()=>{playing=false;if(!current())return;
const r=await fetch('/api/decision',{method:'POST',headers:{'Content-Type':'application/json','X-Review-Token':token},
body:JSON.stringify({episode:current().episode,decision,operator:$('operator').value,note:$('note').value})});
$('result').textContent=await r.text();if(r.ok){current().review=decision;$('episodes').selectedOptions[0].textContent=`${current().episode} | ${current().status} | ${decision}`}};
refresh();</script></html>"""


def make_review_server(audit_dir, rules, host="127.0.0.1", port=8765):
    if host not in ("127.0.0.1", "localhost", "::1"):
        raise ValueError("人工审核服务仅绑定本机")
    audit_dir = Path(audit_dir).resolve()
    decision_path = audit_dir / "review_decisions.json"
    decisions = (
        json.loads(decision_path.read_text(encoding="utf-8")) if decision_path.exists() else {}
    )
    token = secrets.token_urlsafe(32)
    lock = threading.Lock()

    def read_reports():
        path = audit_dir / "audit_report.json"
        if not path.exists():
            return []
        reports = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(reports, list):
            raise ValueError("审计报告格式错误，请重新运行 python3 -m data_collect.cli.review")
        return reports

    def sync_for(row):
        path = audit_dir / "sync_indices" / (Path(row["episode"]).stem + "_sync.json")
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, body, content_type="application/json", status=200):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            try:
                query = urlparse(self.path)
                params = parse_qs(query.query)
                if query.path == "/":
                    self.send(
                        HTML.replace("__TOKEN__", token).encode("utf-8"), "text/html; charset=utf-8"
                    )
                    return
                if query.path == "/api/episodes":
                    result = []
                    for row in read_reports():
                        sync = sync_for(row)
                        frames = (
                            len(sync["grid_host_ts_ns"])
                            if sync
                            else max(1, int(row.get("duration_s", 1) * 20))
                        )
                        result.append(
                            {
                                **row,
                                "frames": frames,
                                "review": decisions.get(row["episode"], {}).get(
                                    "decision", "pending"
                                ),
                            }
                        )
                    self.send(result)
                    return
                by_name = {r["episode"]: r for r in read_reports()}
                row = by_name[params["episode"][0]]
                k = int(params["k"][0])
                sync = sync_for(row)
                try:
                    import cv2
                    import h5py
                    import numpy as np
                except ImportError as exc:
                    raise RuntimeError(
                        "缺少数据预览依赖，请在当前 Python 环境中进入 Data_Collect "
                        "并执行 python3 -m pip install -e ."
                    ) from exc
                with h5py.File(row["path"], "r") as f:
                    if query.path == "/api/frame":
                        stream = params["stream"][0]
                        if stream not in IMAGE_STREAMS:
                            raise ValueError("unknown stream")
                        if sync:
                            index = sync["indices"][stream][k]
                        else:
                            ts = f[stream + "/available_host_ts_ns"][:]
                            t = int(f.attrs["start_host_ts_ns"]) + k * 50_000_000
                            index = int(
                                np.clip(np.searchsorted(ts, t, side="right") - 1, 0, len(ts) - 1)
                            )
                        with EpisodeImages(f) as images:
                            rgb = images.read(stream, index)
                        ok, image = cv2.imencode(".jpg", rgb[..., ::-1], [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if not ok:
                            raise ValueError("preview encode failed")
                        self.send(image.tobytes(), "image/jpeg")
                    elif query.path == "/api/state":
                        if sync:
                            i = sync["indices"]["robot_feedback"][k]
                            j = sync["indices"]["robot_command"][k]
                            self.send(
                                {
                                    "joint_pos": f["robot_feedback/joint_pos"][i].tolist(),
                                    "gripper_pos": f["robot_feedback/gripper_pos"][i].tolist(),
                                    "target_joint_pos": f["robot_command/target_joint_pos"][
                                        j
                                    ].tolist(),
                                    "target_gripper_pos": f["robot_command/target_gripper_pos"][
                                        j
                                    ].tolist(),
                                }
                            )
                        else:
                            self.send({"preview": "自动 REJECT：原始异步帧预览，仅供诊断"})
                    else:
                        self.send({"error": "not found"}, status=404)
            except Exception as exc:
                self.send({"error": str(exc)}, status=400)

        def do_POST(self):
            try:
                if self.path != "/api/decision" or self.headers.get("X-Review-Token") != token:
                    self.send({"error": "invalid request"}, status=403)
                    return
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 16384:
                    raise ValueError("invalid request size")
                data = json.loads(self.rfile.read(length))
                by_name = {r["episode"]: r for r in read_reports()}
                row = by_name[data["episode"]]
                decision = data["decision"]
                if (
                    decision not in ("approve", "reject", "pending")
                    or not data.get("operator", "").strip()
                ):
                    raise ValueError("请选择有效决定并填写审核人")
                if decision == "approve":
                    if row["status"] != "PASS" or not sync_for(row):
                        raise ValueError("自动 REJECT 或缺少索引，无法人工放行")
                    if fingerprint(row["path"]) != row["source_sha256"]:
                        raise ValueError("原文件已变化，请重新自动审计")
                    import h5py
                    with h5py.File(row["path"], "r") as file:
                        validate_images(file)
                with lock:
                    from data_collect.dataset.audit import build_manifest

                    decisions[row["episode"]] = {
                        "decision": decision,
                        "operator": data["operator"],
                        "note": data.get("note", ""),
                        "source_sha256": row.get("source_sha256"),
                        "reviewed_at": datetime.now(timezone.utc).isoformat(),
                    }
                    atomic_json(decision_path, decisions)
                    manifest = build_manifest(audit_dir, rules)
                self.send({"decision": decision, "approved_episodes": len(manifest["episodes"])})
            except Exception as exc:
                self.send({"error": str(exc)}, status=400)

    return ThreadingHTTPServer((host, port), Handler)


def serve_review(audit_dir, rules, host="127.0.0.1", port=8765):
    server = make_review_server(audit_dir, rules, host, port)
    print(f"打开 http://{host}:{server.server_port} 审核；Ctrl-C 退出", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
