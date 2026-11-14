#!/usr/bin/env python3
"""Render original GoPro frames beside synchronized SLAM-derived TCP trajectory."""
import argparse
import csv
import json
import pickle
import subprocess
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def make_3d_panel(coordinates):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from mpl_toolkits.mplot3d import proj3d

    points = coordinates * 1000
    valid = np.isfinite(points).all(axis=1)
    low = np.min(points[valid], axis=0)
    high = np.max(points[valid], axis=0)
    center = (low + high) / 2
    half_range = max(float(np.max(high-low)) * 0.60, 20.0)
    fig = plt.figure(figsize=(4.3, 4.9), dpi=100, facecolor='#292f37')
    ax = fig.add_axes([0.02, 0.07, 0.87, 0.82], projection='3d', facecolor='#292f37')
    for i, setter in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
        setter(center[i]-half_range, center[i]+half_range)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=24, azim=-54)
    ax.set_xlabel('X (mm)', color='white', labelpad=1)
    ax.set_ylabel('Y (mm)', color='white', labelpad=1)
    ax.set_zlabel('Z (mm)', color='white', labelpad=0)
    ax.tick_params(colors='white', labelsize=7, pad=0)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_pane_color((.19, .21, .24, 1.))
    ax.plot(points[:, 0], points[:, 1], points[:, 2], color='#77808a', linewidth=1)
    fig.text(.04, .96, 'TCP trajectory: 3D SLAM coordinates', color='white', fontsize=10, va='top')
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    background = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    px = np.zeros((len(points), 2), dtype=np.int32)
    x2, y2, _ = proj3d.proj_transform(points[valid, 0], points[valid, 1], points[valid, 2], ax.get_proj())
    screen = ax.transData.transform(np.column_stack((x2, y2)))
    screen[:, 1] = background.shape[0] - screen[:, 1]
    px[valid] = np.rint(screen).astype(np.int32)
    plt.close(fig)
    return background, px, valid


def draw_3d_panel(canvas, background, points, valid, frame_idx):
    x, y = 830, 75
    h, w = background.shape[:2]
    panel = background.copy()
    indexes = np.flatnonzero(valid[:frame_idx+1])
    for a, b in zip(indexes[:-1], indexes[1:]):
        if b == a+1:
            cv2.line(panel, tuple(points[a]), tuple(points[b]), (80, 185, 240), 2, cv2.LINE_AA)
    first = np.flatnonzero(valid)[0]
    cv2.circle(panel, tuple(points[first]), 5, (80, 210, 100), -1, cv2.LINE_AA)
    if valid[frame_idx]:
        cv2.circle(panel, tuple(points[frame_idx]), 7, (70, 75, 255), -1, cv2.LINE_AA)
    canvas[y:y+h, x:x+w] = panel


def panel(canvas, rect, coordinates, frame_idx, axes, title):
    x, y, w, h = rect
    cv2.rectangle(canvas, (x, y), (x+w, y+h), (41, 47, 55), -1)
    cv2.putText(canvas, title, (x+12, y+23), cv2.FONT_HERSHEY_SIMPLEX, .55, (235,235,235), 1, cv2.LINE_AA)
    p = coordinates[:, axes] * 1000
    valid = np.isfinite(p).all(axis=1)
    if not np.any(valid):
        return
    low = np.nanmin(p, axis=0); high = np.nanmax(p, axis=0)
    center = (low + high) / 2
    span = np.maximum(high-low, 30.) * 1.2
    draw_w, draw_h = w - 76, h - 70
    scale = min(draw_w/span[0], draw_h/span[1])
    pts = np.zeros((len(p), 2), dtype=np.int32)
    pts[valid, 0] = np.rint(x+w/2 + (p[valid,0]-center[0])*scale).astype(np.int32)
    pts[valid, 1] = np.rint(y+38+draw_h/2 - (p[valid,1]-center[1])*scale).astype(np.int32)
    for a,b in zip(np.flatnonzero(valid)[:-1],np.flatnonzero(valid)[1:]):
        if b==a+1:cv2.line(canvas,tuple(pts[a]),tuple(pts[b]),(92,100,108),1,cv2.LINE_AA)
    for a,b in zip(np.flatnonzero(valid[:frame_idx+1])[:-1],np.flatnonzero(valid[:frame_idx+1])[1:]):
        if b==a+1:cv2.line(canvas,tuple(pts[a]),tuple(pts[b]),(80,185,240),2,cv2.LINE_AA)
    first=np.flatnonzero(valid)[0]
    cv2.circle(canvas,tuple(pts[first]),5,(80,210,100),-1,cv2.LINE_AA)
    if valid[frame_idx]:cv2.circle(canvas,tuple(pts[frame_idx]),7,(70,75,255),-1,cv2.LINE_AA)
    cv2.putText(canvas, 'start', (x+12,y+h-13), cv2.FONT_HERSHEY_SIMPLEX,.38,(80,210,100),1,cv2.LINE_AA)
    cv2.putText(canvas, 'current', (x+58,y+h-13), cv2.FONT_HERSHEY_SIMPLEX,.38,(70,75,255),1,cv2.LINE_AA)
    cv2.putText(canvas, f'scale: {draw_w/scale:.0f} mm wide', (x+w-175,y+h-13), cv2.FONT_HERSHEY_SIMPLEX,.36,(180,185,190),1,cv2.LINE_AA)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--episode', type=int, required=True, help='one-based episode index')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--tcp', default='0,0.096,0.138', help='camera-to-TCP translation in meters')
    ap.add_argument('--plot-3d', action='store_true', help='render a fixed-view 3D trajectory')
    args=ap.parse_args()
    tcp=np.asarray([float(x) for x in args.tcp.split(',')]);
    if tcp.shape!=(3,) or not np.isfinite(tcp).all():ap.error('TCP must have three finite coordinates')
    plan=pickle.load((args.session/'dataset_plan.pkl').open('rb'))
    if args.episode<1 or args.episode>len(plan):ap.error('episode out of range')
    ep=plan[args.episode-1]; cam=ep['cameras'][0];vp=args.session/'demos'/cam['video_path'];rows=list(csv.DictReader((vp.parent/'camera_trajectory.csv').open()))
    capture=cv2.VideoCapture(str(vp));fps=capture.get(cv2.CAP_PROP_FPS);count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if not capture.isOpened() or fps<=0 or count<=0:raise RuntimeError('Cannot open video')
    pos=np.full((count,3),np.nan)
    for row in rows:
        i=int(row['frame_idx']); q=np.array([float(row[k]) for k in ('q_x','q_y','q_z','q_w')]);p=np.array([float(row[k]) for k in ('x','y','z')])
        if 0<=i<count and row['is_lost'].lower()=='false' and np.isfinite(q).all() and np.isfinite(p).all() and np.linalg.norm(q)>.5:
            pos[i]=p+Rotation.from_quat(q).apply(tcp)
    if not np.isfinite(pos).all(axis=1).any():raise RuntimeError('No valid camera poses')
    first=np.flatnonzero(np.isfinite(pos).all(axis=1))[0];rel=pos-pos[first]
    if args.plot_3d:
        background_3d, projected_3d, valid_3d = make_3d_panel(rel)
    start,end=cam['video_start_end']; widths=np.asarray(ep['grippers'][0]['gripper_width'])
    if len(widths)!=end-start:raise RuntimeError('Gripper data and video frame range differ')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','1280x720','-r',f'{fps:.12f}','-i','-','-an','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
    n=0
    try:
        for i in range(count):
            ok,frame=capture.read()
            if not ok:raise RuntimeError(f'Video ended at frame {i}, expected {count}')
            canvas=np.full((720,1280,3),(24,28,33),np.uint8)
            cv2.putText(canvas,f'EPISODE {args.episode}  |  ORIGINAL VIDEO + SLAM-DERIVED TCP', (20,34),cv2.FONT_HERSHEY_SIMPLEX,.75,(240,240,240),2,cv2.LINE_AA)
            canvas[68:668,10:810]=cv2.resize(frame,(800,600),interpolation=cv2.INTER_AREA)
            cv2.rectangle(canvas,(820,68),(1270,668),(35,40,46),-1)
            if args.plot_3d:
                draw_3d_panel(canvas, background_3d, projected_3d, valid_3d, i)
            else:
                panel(canvas,(830,75,430,238),rel,i,(0,1),'TCP trajectory: SLAM X-Y (mm)')
                panel(canvas,(830,320,430,238),rel,i,(0,2),'TCP trajectory: SLAM X-Z (mm)')
            if np.isfinite(rel[i]).all():
                x,y,z=rel[i]*1000
                msg=f'Relative XYZ: {x:+.1f}, {y:+.1f}, {z:+.1f} mm'
            else:msg='Tracking unavailable'
            cv2.putText(canvas,msg,(835,585),cv2.FONT_HERSHEY_SIMPLEX,.45,(230,230,230),1,cv2.LINE_AA)
            width=f'{widths[i-start]*1000:.1f} mm' if start<=i<end else 'not in dataset interval'
            cv2.putText(canvas,'Estimated gripper opening: '+width,(835,612),cv2.FONT_HERSHEY_SIMPLEX,.45,(230,230,230),1,cv2.LINE_AA)
            cv2.putText(canvas,f't={i/fps:.2f}s  frame={i+1}/{count}',(835,640),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1,cv2.LINE_AA)
            cv2.putText(canvas,'Gray: complete estimated path  |  orange: path so far  |  red: current', (15,700),cv2.FONT_HERSHEY_SIMPLEX,.53,(210,210,210),1,cv2.LINE_AA)
            proc.stdin.write(canvas.tobytes());n+=1
        proc.stdin.close()
        if proc.wait()!=0:raise RuntimeError('ffmpeg failed')
    finally:
        capture.release()
        if proc.poll() is None:
            proc.stdin.close();proc.terminate();proc.wait()
    print(json.dumps({'output':str(args.output),'frames':n,'fps':fps,'valid_tcp_poses':int(np.isfinite(pos).all(axis=1).sum()),'gripper_frame_interval':[start,end]}))

if __name__=='__main__':main()
