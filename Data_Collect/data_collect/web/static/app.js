import {createRobotView} from './robot.js';
const $=id=>document.getElementById(id), deg=x=>x*180/Math.PI;
const colors=['#47d6bb','#73acff','#c49bff','#f5bd6e','#fa839d','#70d4f5'];
const labels={'cameras/global_rgb':['全局相机','D455'],'cameras/wrist_rgb':['手腕相机','D435i'],'tactile/left_raw':['左触觉','Tac-X'],'tactile/right_raw':['右触觉','Tac-X']};
let boot,viewer=null,state=null,logCursor=0,logs=[],samples=[],previous=null,chartMode='position',pendingAction=false,lastRobotStamp=0,localMessage='',lastPoll=0;
const number=(value,digits=1)=>typeof value==='number'&&Number.isFinite(value)?value.toFixed(digits):'—';
function text(id,value){$(id).textContent=value}
function fault(value){return Array.isArray(value)?value.filter(Boolean).join('；'):typeof value==='string'?value:''}
function toast(message){text('toast',message);$('toast').hidden=false;clearTimeout(toast.timer);toast.timer=setTimeout(()=>$('toast').hidden=true,6000)}
function phase(label,kind=''){text('phase',label);$('phase').className='phase '+kind;$('run-indicator').className='run-indicator '+kind}
function fit(canvas){const r=canvas.getBoundingClientRect(),scale=Math.min(devicePixelRatio,2);if(canvas.width!==Math.round(r.width*scale)||canvas.height!==Math.round(r.height*scale)){canvas.width=Math.round(r.width*scale);canvas.height=Math.round(r.height*scale)}const ctx=canvas.getContext('2d');ctx.setTransform(scale,0,0,scale,0,0);return {ctx,w:r.width,h:r.height}}
function prepareUI(){
 for(const [stream,[name,type]] of Object.entries(labels)){
  const row=document.createElement('div');row.className='sensor';row.dataset.stream=stream;
  row.innerHTML='<i class="dot"></i><div><strong></strong><small></small></div><span class="rate">— Hz</span>';
  row.title='帧率按近2秒真实帧统计；延迟为驱动采样时的帧龄';row.querySelector('strong').textContent=name;row.querySelector('small').textContent=type+' · 等待连接';$('sensors').append(row);
 }
 for(const [name,label] of Object.entries(boot.nodes)){
  const row=document.createElement('div');row.className='node';row.dataset.node=name;row.innerHTML='<i class="dot"></i><span></span><span>未发现</span>';row.children[1].textContent=label;$('nodes').append(row);
 }
 const joints=boot.model.joints.filter(j=>/^joint[1-6]$/.test(j.name));
 joints.forEach((joint,i)=>{
  const row=document.createElement('div');row.className='joint-row';row.dataset.joint=i;row.innerHTML=`<div class="joint-name"><span>J${i+1}</span><div class="limit-track"><i></i></div></div><span class="actual">—</span><span class="target">—</span><span class="velocity">—</span><span class="error">—</span><canvas class="spark"></canvas>`;
  row.querySelector('.joint-name').title=`URDF 限位 ${number(deg(joint.limits[0]))}° ～ ${number(deg(joint.limits[1]))}°`;$('joints').append(row);
  const label=document.createElement('span');label.innerHTML=`<i style="background:${colors[i]}"></i>J${i+1}`;$('joint-legend').append(label);
 });
 for(const name of ['X','Y','Z','Roll','Pitch','Yaw']){const cell=document.createElement('div');cell.className='tcp-value';cell.innerHTML='<small></small><strong>—</strong>';cell.firstChild.textContent=name;$('tcp-values').append(cell)}
 text('speed-limit',number(deg(boot.safety.max_velocity_rad_s),0)+'°/s');text('step-limit',number(deg(boot.safety.max_command_step_rad),1)+'°');text('acceleration-limit','加速度 ≤ '+number(deg(boot.safety.max_acceleration_rad_s2),0)+'°/s² · 目标距离 ≤ '+number(deg(boot.safety.max_target_lead_rad),0)+'°');
 if(boot.demo){$('demo-banner').hidden=false;text('mode-badge','离线演示 / 不连接硬件');text('start','▶ 开始离线演示');text('stop','■ 停止演示');text('hold','Ⅱ 暂停演示运动');text('control-note','演示回零、遥操和录制状态。停止运动可暂停演示；模拟轨迹不会写入数据目录。')}
}
async function action(name){
 if(pendingAction)return;pendingAction=true;updateButtons();
 try{const response=await fetch('/api/'+name,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':document.querySelector('meta[name=control-token]').content},body:JSON.stringify({can_port:$('can-port').value})});const result=await response.json();if(!response.ok)throw Error(result.error||'操作失败');toast(name==='start'?(boot.demo?'演示已启动':'程序正在启动，准备就绪后请按手柄 A 回零对齐'):name==='hold'?'已请求停止运动 / 取消回零':'正在停止进程并等待数据封盘');localMessage='';if(name==='start'){samples=[];previous=null;lastRobotStamp=0;viewer?.clearPath()}}
 catch(error){localMessage=error.message;toast(error.message)}finally{pendingAction=false;updateButtons()}
}
function updateButtons(){const p=state?.process,age=state?.collector_age_ms,online=age!=null&&age<600,external=state?.nodes?.includes('agx_arm_ctrl_single_node');$('start').disabled=pendingAction||!!p?.running||!!p?.stopping||online||external;$('stop').disabled=pendingAction||!p?.running||!!p?.stopping;$('hold').disabled=pendingAction||!(online||p?.running||external);$('can-port').disabled=pendingAction||!!p?.running||online}
function visibleLogs(){return logs.filter(row=>!$('log-errors').checked||/error|failed|exception|warning|warn|故障|失败|异常|拒绝|超时|停止|取消/i.test(row.text))}
async function copyLogs(){
 const rows=visibleLogs();if(!rows.length){toast('当前没有可复制的日志');return}
 const content=rows.map(row=>row.time+' '+row.text).join('\n');
 try{
  try{await navigator.clipboard.writeText(content)}
  catch{
   const field=document.createElement('textarea'),focused=document.activeElement;
   field.value=content;field.readOnly=true;field.style.cssText='position:fixed;left:-9999px;top:0';document.body.append(field);
   try{field.select();if(!document.execCommand('copy'))throw Error('剪贴板不可用')}
   finally{field.remove();focused?.focus({preventScroll:true})}
  }
  toast('已复制 '+rows.length+' 条日志');
 }catch{toast('复制失败，请选中日志后按 Ctrl+C')}
}
function renderLogs(){
 const box=$('logs'),errorsOnly=$('log-errors').checked,rows=visibleLogs();$('copy-log').disabled=!rows.length;
 box.replaceChildren();if(!rows.length){const empty=document.createElement('div');empty.className='log-empty';empty.textContent=errorsOnly?'暂无异常日志':'启动后的进程输出会显示在这里，完整日志保存到 data/logs/gui/。';box.append(empty)}
 for(const row of rows){const line=document.createElement('div');line.className='log-line'+(/error|failed|exception|故障|失败|异常/i.test(row.text)?' error':/warn|拒绝|超时|停止|取消/i.test(row.text)?' warn':'');const time=document.createElement('span');time.className='log-time';time.textContent=row.time;line.append(time,document.createTextNode(row.text));box.append(line)}
 if($('log-follow').checked)box.scrollTop=box.scrollHeight;
}
function update(s){
 state=s;lastPoll=performance.now();const c=s.collector??{},r=c.robot??{},p=s.process??{},age=s.collector_age_ms;
 const online=age!=null&&age<600,fresh=online&&r.age_ms!=null&&r.age_ms+age<300;
 const q=Array.isArray(r.joint_pos)&&r.joint_pos.length===6?r.joint_pos:null;
 const goal=Array.isArray(r.target_joint_pos)&&r.target_joint_pos.length===6?r.target_joint_pos:null;
 const err=(online?fault(c.error):'')||s.bridge_error||p.error||'';
 let title='等待启动',subtitle='界面就绪，机械臂尚未启动';
 if(p.stopping){phase('正在停止','warn');title='正在停止程序';subtitle='等待线程退出和数据写盘完成'}
 else if(err){phase('故障 / 已停止','bad');title='需要检查';subtitle='停止运动后，请查看故障原因与日志'}
 else if(s.demo&&s.demo_paused&&p.running){phase('演示已暂停','warn');title='离线演示已暂停';subtitle='停止演示后可以重新开始'}
 else if(online&&c.preparing_home){phase('等待设备就绪','warn');title='A · 等待设备就绪';subtitle='最多等待 1 秒，等待期间不发送运动指令；B 可取消'}
 else if(online&&c.homing){phase('回零中','warn');title='A · 正在回零';subtitle='低速回零确认后，锁定手柄参考'}
 else if(online&&c.recording){phase('遥操 · 录制中','good');title='运动与数据采集进行中';subtitle='B 或“停止运动”可结束当前轨迹'}
 else if(online&&(c.armed||c.teleop_enabled)){phase('已对齐 / 等待运动','good');title='参考对齐已完成';subtitle='手柄运动后开始录制'}
 else if(online&&(!c.control_ready||!q)){phase('设备未就绪','warn');title='等待机械臂反馈';subtitle='检查 CAN 和设备连接后再按手柄 A'}
 else if(online){phase(c.arm_enabled?'等待手柄 A':'未使能',c.arm_enabled?'good':'warn');title=c.arm_enabled?'机械臂已使能':'机械臂尚未使能';subtitle=p.running?'按手柄 A 回零并建立参考':'外部程序运行中 · 可观察及停止运动'}
 else if(p.running){phase('正在启动','warn');title='等待采集驱动反馈';subtitle='检查 CAN、Quest 和传感器连接'}
 else if(p.returncode!=null){phase(p.returncode===0?'已停止':'进程已退出',p.returncode===0?'':'bad');title='程序已退出';subtitle='启动进程返回码 '+p.returncode}
 else phase('未启动');
 text('run-title',title);text('run-subtitle',subtitle);
 const message=localMessage||err||(p.running&&!online&&s.server_time-(p.started_at||s.server_time)>15?'启动超过 15 秒仍无采集状态，请检查进程日志。':'');$('alert').hidden=!message;text('alert',message);
 text('feedback-age',online?'状态 '+number(age,0)+' ms':age!=null?'反馈已断开':'等待反馈');
 $('connection-dot').className='dot good';text('connection','界面服务已连接');text('clock',new Date(s.server_time*1000).toLocaleTimeString('zh-CN',{hour12:false}));
 for(const row of document.querySelectorAll('.node')){const present=s.nodes.includes(row.dataset.node);row.firstChild.className='dot '+(present?'good':'');row.lastChild.textContent=!present?'未发现':row.dataset.node==='pub_pose_node'?(s.quest_age_ms!=null&&s.quest_age_ms<500?'有位姿':'等待位姿'):row.dataset.node==='agx_arm_ctrl_single_node'&&!online?'等待状态':'已发现'}
 let ready=0;for(const row of document.querySelectorAll('.sensor')){const stream=c.streams?.[row.dataset.stream],a=stream?.age_ms!=null&&online?stream.age_ms+Math.max(0,age-150):null,threshold=row.dataset.stream.startsWith('cameras/')?68:55;const good=a!=null&&a<=threshold;if(good)ready++;row.firstChild.className='dot '+(good?'good':a!=null?'bad':'');row.querySelector('small').textContent=labels[row.dataset.stream][1]+' · '+(a==null?'无数据':number(a,0)+' ms');row.querySelector('.rate').textContent=online&&stream?number(a!=null&&a<1000?stream.fps:0,1)+' Hz':'— Hz'}text('sensor-count',ready+' / 4');
 $('joints').classList.toggle('offline-data',!fresh);
 let velocity=null;
 if(q&&fresh&&r.joint_host_ts_ns!==lastRobotStamp){
  const t=r.joint_host_ts_ns/1e9,dt=previous?t-previous.t:0;
  velocity=previous&&dt>.005&&dt<.5?q.map((v,i)=>(v-previous.q[i])/dt):null;
  samples.push({t:performance.now()/1000,q:q.map(deg),goal:goal?.map(deg)??null,velocity:velocity?.map(deg)??null,error:goal?goal.map((v,i)=>deg(v-q[i])):null});
  const cutoff=performance.now()/1000-20;samples=samples.filter(v=>v.t>=cutoff).slice(-260);previous={t,q:[...q],velocity};lastRobotStamp=r.joint_host_ts_ns;
 }else if(fresh)velocity=previous?.velocity??null;
 else previous=null;
 const limits=boot.model.joints.filter(j=>/^joint[1-6]$/.test(j.name));
 document.querySelectorAll('.joint-row').forEach((row,i)=>{
  row.querySelector('.actual').textContent=q?number(deg(q[i])):'—';row.querySelector('.target').textContent=goal?number(deg(goal[i])):'—';row.querySelector('.velocity').textContent=fresh&&velocity?number(deg(velocity[i])):'—';row.querySelector('.error').textContent=q&&goal?number(deg(goal[i]-q[i])):'—';
  const marker=row.querySelector('.limit-track i'),[lo,hi]=limits[i].limits;marker.style.left=q?Math.max(0,Math.min(100,(q[i]-lo)/(hi-lo)*100))+'%':'50%';marker.style.background=q&&(q[i]<lo-1e-6||q[i]>hi+1e-6)?'#ff7f86':'#40d5bd';row.querySelector('.error').classList.toggle('bad',q&&goal&&Math.abs(goal[i]-q[i])>boot.safety.max_target_lead_rad);
 });
 [...$('tcp-values').children].forEach((cell,i)=>{const v=r.tcp_pose?.[i];cell.lastChild.textContent=typeof v==='number'?number(i<3?v*1000:deg(v),1)+(i<3?' mm':' °'):'—'});
 text('gripper','夹爪 '+number(r.gripper_pos?.[0]*1000,1)+' mm');text('fk-error',online?number(c.fk_position_error_m*1000,2)+' mm':'— mm');
 text('pose-freshness',fresh?'实时反馈 · '+number(r.age_ms+age,0)+' ms':boot.demo?'演示未运行 · 模型预览':q?'反馈已断开 · 显示最后姿态':'无实时数据 · 零位预览');$('pose-freshness').className='pose-freshness '+(fresh?'good':'');
 viewer?.update(q??[0,0,0,0,0,0],goal,r.gripper_pos?.[0]??.04,fresh,r.joint_host_ts_ns??0);
 const recording=online&&c.recording;text('episode',recording?String(c.recording).split('/').pop():c.homing&&online?'回零中 · 暂不录制':'未录制');
 const seconds=recording?c.recording_duration_s||0:0;text('duration',String(Math.floor(seconds/60)).padStart(2,'0')+':'+(seconds%60).toFixed(1).padStart(4,'0'));
 const mb=online?c.writer?.pending_mb||0:0,capacity=c.writer?.capacity_mb||256;text('queue-value',number(mb,1)+' / '+number(capacity,0)+' MiB');$('queue-fill').style.width=Math.min(100,mb/capacity*100)+'%';$('queue-fill').style.background=Math.max(mb/capacity,c.writer?.worst_fill||0)>.8?'#ff7f86':'#40d5bd';
 text('record-note',online&&c.writer?.finalizing?'正在封盘 '+c.writer.finalizing+' 条轨迹':recording?'原始图像与机器人状态异步写盘':'A 回零期间不录制');text('pid',p.pid?'launch PID '+p.pid:boot.demo?'离线演示 · 不启动真实进程':p.log_path?'完整日志已保存':online?'外部程序 · 不由本界面管理':'尚无启动进程');$('pid').title=p.log_path||'';
 const newLogs=(p.logs??[]).filter(row=>row.id>logCursor);logCursor=Math.max(logCursor,p.log_cursor??0);if(newLogs.length){logs.push(...newLogs);logs=logs.slice(-500);renderLogs()}
 updateButtons();drawCharts();
}
function drawSeries(ctx,points,key,j,color,w,h,min,max,left=0,top=0,right=0,bottom=0,dashed=false){
 const latest=performance.now()/1000,end=latest,start=latest-20;
 ctx.strokeStyle=color;ctx.lineWidth=1.3;ctx.setLineDash(dashed?[3,3]:[]);ctx.beginPath();let pen=false;
 for(const s of points){const y=s[key]?.[j];if(!Number.isFinite(y)){pen=false;continue}const x=left+(s.t-start)/20*(w-left-right),py=top+(max-y)/(max-min)*(h-top-bottom);if(pen)ctx.lineTo(x,py);else ctx.moveTo(x,py);pen=true}ctx.stroke();ctx.setLineDash([]);
}
function drawCharts(){
 const {ctx,w,h}=fit($('chart'));if(!w||!h)return;ctx.clearRect(0,0,w,h);
 const key=chartMode==='position'?'q':chartMode;const values=samples.flatMap(s=>s[key]??[]).filter(Number.isFinite);let min=values.length?Math.min(...values):chartMode==='position'?-10:-5,max=values.length?Math.max(...values):chartMode==='position'?10:5;
 if(chartMode!=='position'){min=Math.min(min,0);max=Math.max(max,0)}const pad=Math.max((max-min)*.15,chartMode==='position'?1:.3);min-=pad;max+=pad;
 ctx.font='9px ui-monospace,monospace';ctx.lineWidth=1;const left=37,right=7,top=12,bottom=23;
 for(let i=0;i<5;i++){const y=top+(h-top-bottom)*i/4;ctx.strokeStyle='#26364a';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(w-right,y);ctx.stroke();ctx.fillStyle='#728ca6';ctx.textAlign='right';ctx.fillText(number(max-(max-min)*i/4,1),left-6,y+3)}
 ctx.textAlign='center';for(let i=0;i<=4;i++){const x=left+(w-left-right)*i/4;ctx.fillStyle='#657f9b';ctx.fillText(i===4?'现在':String(-20+i*5)+'s',x,h-5)}
 if(samples.length<2){ctx.textAlign='center';ctx.fillStyle='#6f86a0';ctx.fillText('等待实时关节反馈',w/2,h/2)}else for(let j=0;j<6;j++)drawSeries(ctx,samples,key,j,colors[j],w,h,min,max,left,top,right,bottom);
 for(const [j,canvas] of [...document.querySelectorAll('.spark')].entries()){const {ctx,w,h}=fit(canvas);if(!w||!h)continue;ctx.clearRect(0,0,w,h);const values=samples.flatMap(s=>[s.q?.[j],s.goal?.[j]]).filter(Number.isFinite);const lo=values.length?Math.min(...values)-.5:-1,hi=values.length?Math.max(...values)+.5:1;drawSeries(ctx,samples,'q',j,'#d9e4f0',w,h,lo,hi,0,2,0,2);drawSeries(ctx,samples,'goal',j,'#40d5bd',w,h,lo,hi,0,2,0,2,true)}
}
async function poll(){
 try{const response=await fetch('/api/state?after_log='+logCursor,{signal:AbortSignal.timeout(2500)});if(!response.ok)throw Error('状态接口异常');update(await response.json())}
 catch(error){$('connection-dot').className='dot bad';text('connection','界面服务断开');phase('连接中断','bad');text('feedback-age','数据已断开');text('pose-freshness','连接中断 · 显示最后姿态');$('pose-freshness').className='pose-freshness';$('joints').classList.add('offline-data');$('start').disabled=true;$('hold').disabled=true;$('stop').disabled=true;if(state?.collector?.robot?.joint_pos)viewer?.update(state.collector.robot.joint_pos,null,state.collector.robot.gripper_pos?.[0]??.04,false);$('alert').hidden=false;text('alert','界面服务暂不可达。请检查运行 gui.py 的终端；停止运动仍可使用手柄 B 或硬件急停。')}
 finally{setTimeout(poll,100)}
}
async function init(){
 try{const response=await fetch('/api/boot');if(!response.ok)throw Error('配置读取失败');boot=await response.json();prepareUI();
 $('start').onclick=()=>action('start');$('hold').onclick=()=>action('hold');$('stop').onclick=()=>action('stop');
 $('clear-chart').onclick=()=>{samples=[];previous=null;viewer?.clearPath();drawCharts()};$('copy-log').onclick=copyLogs;$('clear-log').onclick=()=>{logs=[];renderLogs()};$('log-errors').onchange=renderLogs;
 for(const button of document.querySelectorAll('[data-chart]'))button.onclick=()=>{chartMode=button.dataset.chart;document.querySelectorAll('[data-chart]').forEach(b=>b.classList.toggle('active',b===button));text('chart-unit',chartMode==='velocity'?'°/s':'°');drawCharts()};
 for(const button of document.querySelectorAll('[data-view]'))button.onclick=()=>{viewer?.view(button.dataset.view);document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b===button))};
 $('show-target').onchange=()=>viewer?.showTarget($('show-target').checked);$('show-path').onchange=()=>viewer?.showPath($('show-path').checked);new ResizeObserver(drawCharts).observe($('chart'));
 createRobotView($('viewport'),boot.model).then(v=>{viewer=v;viewer.showTarget($('show-target').checked);viewer.showPath($('show-path').checked);if(state)update(state)}).catch(error=>{text('model-status','三维模型不可用：'+error.message+'；数值和状态监测仍可使用。');$('model-status').style.color='#f3b764';console.error(error)});
 poll();
 }catch(error){$('alert').hidden=false;text('alert','界面初始化失败：'+error.message);console.error(error)}
}
init();
