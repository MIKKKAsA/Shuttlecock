# -*- coding: utf-8 -*-
import math

NET_X, NET_H = 6.7, 1.5
G = 9.8
# ===== 身体位置 =====
HIP_X, HIP_Y = -0.60, 0.85        # 髋：控制整体位移
THIGH_L = 0.25                    # 大腿固定长度
LEG_ANG = 45                      # 击球时大腿相对髋垂线角度（°），0=竖直；准备时为 0
# ===== 膝盖弯曲（驱动击球瞬间）=====
BETA_SET = 90                     # 击球时膝盖角：0=折叠垂直，90=伸直直线（不可超过）；准备时为 0
# ===== 腿/板尺寸 =====
L1, L2 = 0.55, 0.24
BOARD_T, BR = 0.006, 0.055
T = 0.30
E_REST, MU = 0.4, 0.15
DRAG = 0.8
BALL_Y = 0.25
BALL_REL = 0.60

def knee():
    a=math.radians(LEG_ANG)
    return (HIP_X+THIGH_L*math.sin(a), HIP_Y-THIGH_L*math.cos(a))

def rot(v,a):
    c,s=math.cos(a),math.sin(a)
    return (c*v[0]-s*v[1], s*v[0]+c*v[1])

def phi_abs(theta,beta):           # 小腿绝对方向（相对竖直）= 大腿角 - 90 + 膝盖角
    return math.radians(theta-90+beta)

def pose(phi, ang, kx, ky):
    d1=(math.sin(phi),-math.cos(phi))
    d2=rot(d1, math.radians(90-ang))
    ank=(kx+L1*d1[0], ky+L1*d1[1])
    toe=(ank[0]+L2*d2[0], ank[1]+L2*d2[1])
    return d2,ank,toe

def closest(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]; l2=dx*dx+dy*dy
    u=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/l2)) if l2 else 0
    return (a[0]+u*dx,a[1]+u*dy)

def predict(ang,power,bx,by,kx,ky,beta_set,theta_set):
    Tsw=T*(100.0/power)
    N=1500
    for i in range(N+1):
        u=i/N
        theta=theta_set*u*u
        beta=beta_set*u*u
        # 大腿、小腿联动：总角速度 = (大腿角+膝盖角) 随时间变化
        w=math.radians(theta_set+beta_set)/Tsw*2*u
        phi=phi_abs(theta,beta)
        kx=HIP_X+THIGH_L*math.sin(math.radians(theta))
        ky=HIP_Y-THIGH_L*math.cos(math.radians(theta))
        d2,ank,toe=pose(phi,ang,kx,ky)
        c=closest((bx,by),ank,toe)
        d=math.hypot(bx-c[0],by-c[1])
        if d<=BR+BOARD_T:
            nx=(bx-c[0])/max(d,1e-9);ny=(by-c[1])/max(d,1e-9)
            vs=(-w*(c[1]-ky), w*(c[0]-kx))
            if vs[0]*nx+vs[1]*ny>0:
                vn0=(-vs[0])*nx+(-vs[1])*ny
                vt0=(-vs[0])*d2[0]+(-vs[1])*d2[1]
                vx=vs[0]+(-E_REST*vn0)*nx+(1-MU)*vt0*d2[0]
                vy=vs[1]+(-E_REST*vn0)*ny+(1-MU)*vt0*d2[1]
                return vx,vy,phi,beta
    return None

def simulate(ang=90,power=100,ball_rel=BALL_REL,drag=DRAG,beta_set=None,
             theta_set=None):
    if beta_set is None:beta_set=BETA_SET
    if theta_set is None:theta_set=LEG_ANG
    kx0,ky0=HIP_X,HIP_Y-THIGH_L          # 准备姿态膝（大腿竖直）
    bx,by=kx0+ball_rel,BALL_Y
    pr=predict(ang,power,bx,by,kx0,ky0,beta_set,theta_set)
    if pr is None: return "未踢中"
    vx,vy,phi,beta=pr
    x,y=bx,by
    h=1/2400;netclear=None;nethit=False;apex=y;ft=0
    while True:
        vx+=-drag*vx*h;vy+=(-G-drag*vy)*h
        px,py=x,y;x+=vx*h;y+=vy*h;ft+=h;apex=max(apex,y)
        if netclear is None and px<NET_X and x>=NET_X:
            uu=(NET_X-px)/(x-px);yat=py+uu*(y-py)
            netclear=yat-NET_H
            if yat-BR<=NET_H:nethit=True
        if nethit:x=NET_X;vx=0;vy=-0.6
        if y<=BR:landed=x;break
    verdict='下网' if nethit else('出界' if landed>13.4 else('未过网' if landed<=NET_X else '界内'))
    pv=math.hypot(pr[0],pr[1]);pa=math.degrees(math.atan2(pr[1],pr[0]))
    return dict(pred_ang=pa,pred_spd=pv,netclear=netclear,apex=apex,
                landed=landed,land_from_net=landed-NET_X,flight_t=ft,
                verdict=verdict,hit_beta=beta)

r=simulate()
if isinstance(r,str):print(f"默认: {r}")
else:
 print(f"默认(beta{BETA_SET}): {r['verdict']} | 击球时膝盖角{r['hit_beta']:.1f} 出射{r['pred_ang']:.1f}° 速{r['pred_spd']:.1f} "
      f"网{r['netclear']:+.2f} 距网{r['land_from_net']:.2f} apex{r['apex']:.2f}")
print('--- 摆腿目标膝盖角 BETA_SET ---')
for bs in [60,70,80,85,90]:
    r=simulate(beta_set=bs)
    if isinstance(r,str):print(f"beta{bs}: {r}");continue
    print(f"set{bs}: {r['verdict']} 击球β{r['hit_beta']:.1f} 出射{r['pred_ang']:.1f} 速{r['pred_spd']:.1f} 网{r['netclear']:+.2f} 距网{r['land_from_net']:.2f}")
print('--- 整体位移 HIP_X ---')
for hx in [-1.2,-0.9,-0.6,-0.3,0.0]:
    HIP_X=hx; r=simulate()
    if isinstance(r,str):print(f"hip{hx}: {r}");continue
    print(f"hip{hx:+.2f}: {r['verdict']} 网{r['netclear']:+.2f} 距网{r['land_from_net']:.2f}")
HIP_X=-0.60
print('--- 大腿角度 LEG_ANG (rel0.32) ---')
for la in [24,32,40,48,55]:
    LEG_ANG=la; r=simulate()
    if isinstance(r,str):print(f"ang{la}: {r}");continue
    nc='  —  ' if r['netclear'] is None else f"{r['netclear']:+.2f}"
    print(f"leg{la:2d}°: {r['verdict']} 击球β{r['hit_beta']:.1f} 出射{r['pred_ang']:.1f} 网{nc} 距网{r['land_from_net']:.2f}")
LEG_ANG=24
def mindist(ang,beta_set,theta_set):
    N=1500;md=9
    bx=HIP_X+BALL_REL;by=BALL_Y
    for i in range(N+1):
        u=i/N;theta=theta_set*u*u;beta=beta_set*u*u
        phi=phi_abs(theta,beta)
        kx=HIP_X+THIGH_L*math.sin(math.radians(theta))
        ky=HIP_Y-THIGH_L*math.cos(math.radians(theta))
        d2,ank,toe=pose(phi,ang,kx,ky)
        c=closest((bx,by),ank,toe)
        md=min(md,math.hypot(bx-c[0],by-c[1]))
    return md
print('--- 诊断：β_set 对板-球最小距离 ---')
for bs in [70,75,80,85,90]:
    print(f"set{bs}: minDist={mindist(90,bs,45):.3f} (阈值{BR+BOARD_T:.3f})")
print('--- 球相对膝位置 ---')
for brp in [0.32,0.45,0.50,0.55,0.60]:
    r=simulate(ball_rel=brp)
    if isinstance(r,str):print(f"rel{brp:.2f}: {r}");continue
    nc='  —  ' if r['netclear'] is None else f"{r['netclear']:+.2f}"
    print(f"rel{brp:.2f}: {r['verdict']} 出射{r['pred_ang']:.1f} 速{r['pred_spd']:.1f} 网{nc} 距网{r['land_from_net']:.2f} apex{r['apex']:.2f}")
