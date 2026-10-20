"""Architecture snapshot grounded in the October 15 morning report."""
from pathlib import Path
import argparse
import math
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser()
p.add_argument('--font', required=True)
a = p.parse_args()
im = Image.new('RGB', (2400, 2520), '#f5f8fc')
d = ImageDraw.Draw(im)
blue, orange, ink, gray = '#2263aa', '#bc7029', '#192d45', '#53657a'
def txt(x,y,s,n=29,color=ink):
    d.text((x,y),s,font=ImageFont.truetype(a.font,n),fill=color)
def line(points,planned=False):
    c=orange if planned else blue
    for (x,y),(u,v) in zip(points,points[1:]):
        dist=math.hypot(u-x,v-y)
        if not planned:
            d.line((x,y,u,v),fill=c,width=5)
        else:
            for start in range(0,int(dist),24):
                end=min(start+13,dist)
                d.line((x+(u-x)*start/dist,y+(v-y)*start/dist,x+(u-x)*end/dist,y+(v-y)*end/dist),fill=c,width=5)
def arrow(points,planned=False):
    line(points,planned)
    x,y=points[-1];u,v=points[-2];t=math.atan2(y-v,x-u)
    d.polygon([(x,y),(x-19*math.cos(t-.45),y-19*math.sin(t-.45)),(x-19*math.cos(t+.45),y-19*math.sin(t+.45))],fill=orange if planned else blue)
def panel(x,y,w,h,title):
    d.rounded_rectangle((x,y,x+w,y+h),radius=24,fill='#eaf1fb',outline='#c7d8ef',width=3)
    txt(x+24,y+15,title,34,blue)
def box(x,y,w,h,title,lines=(),planned=False):
    d.rounded_rectangle((x,y,x+w,y+h),radius=15,fill='#fffaf3' if planned else 'white',outline='#dec49f' if planned else '#8baed8',width=3)
    txt(x+18,y+12,title,32,orange if planned else blue)
    for i,s in enumerate(lines):txt(x+18,y+61+37*i,s,27,gray)

txt(60,30,'PAI｜現在のアーキテクチャと進捗',52)
txt(60,105,'2026-10-20 · GitHub checkpoint 9bd2c29 + 10/18〜20 作業記録に基づく。実機再試験は未実施。',28,gray)
line([(70,179),(145,179)]); txt(165,159,'実装・確認した経路（試験条件は下段）',28)
line([(1200,179),(1275,179)],True);txt(1295,159,'未統合・今後の結合',28,orange)

panel(60,235,2280,435,'認識・目標・判断｜Mac / ROS 2・Colima / Gazebo')
box(100,320,555,175,'C270 / Gazebo → YOLO',
    ['HTTP :8765 → 検出JSON','/webcam/detections','実カメラのdry-run試験あり'])
box(755,320,690,175,'床面投影 → 候補選別',
    ['odom候補 → map候補','/litter/target_candidate → target','近接除外・受理／拒否の記録'])
box(1545,320,755,175,'Navigation',
    ['目標と姿勢をmapで照合','旋回・接近・到達停止','/nav/cmd_vel_unstamped など'])
arrow([(655,407),(755,407)]); arrow([(1445,407),(1545,407)])
box(100,535,1345,105,'姿勢：Gazebo / 手動入力で検証済み',
    ['/robot/pose · 10/18実カメラdry-runは手動姿勢を含む'])
arrow([(1445,585),(1475,585),(1475,515),(1920,515),(1920,495)])
txt(1510,550,'ArUco＋オドメトリ：採用方針',29,orange)
txt(1510,593,'対応ソースの同期・実測校正・統合は未完了',27,orange)

panel(60,730,2280,360,'出力A｜認識からの動作を、車輪を動かさず検証')
box(100,815,555,175,'出力先の切替',
    ['Gazebo：Create 3へ接近・停止','dry-run：ROS出力 → bridge','実機自律OIへの結合は未実施'])
box(755,815,690,175,'TCP 9001 / dry-run通信',
    ['session ID・CRC・ACK','再接続・旧非ゼロ指令の拒否','実装内容は10/18作業報告に基づく'])
box(1545,815,755,175,'実Pico W / dry-run受信',
    ['5ケース合格：UART/OIは未使用','対象消失STOP記録 約55 ms','これは車輪の制動時間ではない'])
arrow([(655,902),(755,902)]);arrow([(1445,902),(1545,902)])
txt(100,1030,'9001コードは今回のGitHub treeに未収録。ACK timeout・再接続遅延の改善確認が残る。',28,gray)

panel(60,1150,2280,415,'出力B｜独立した限定試験 → TCP 9000 → Pico W → Roomba 680')
box(100,1235,555,220,'車輪浮上・限定試験ノード',
    ['20 mm/s・最大1秒の試験','roomba_bridge_tcp / transport','ARM generation・sequence','CRC・per-Drive UART ACK'])
box(755,1235,690,220,'実Pico W / pico_relay',
    ['指令の検査 → UART完全書込み','受理Driveでwatchdog更新','STOP・切断・Safe喪失の処理','OI：115200 bps'])
box(1545,1235,755,220,'Roomba 680 / OI Drive Direct',
    ['10/19：浮上で低速回転を確認','watchdog 約503 ms → 左右輪停止','終了後、非ゼロ許可falseへ復帰','床上での自律走行は未実施'])
arrow([(655,1325),(755,1325)]);arrow([(1445,1325),(1545,1325)])
txt(100,1470,'配布版の照合が必要：本日の申告 UART1 / GP4・GP5 ↔ GitHub UART0 / GP0・GP1。',28,orange)
txt(100,1515,'Pico電源断・UART断ではSTOP送信を保証できない。独立給電の完了結果は未確認。',28,gray)

panel(60,1625,2280,430,'今日までの到達点｜試験合格と、次段階の許可を分ける')
rows=[
 ('10/18','PASS','実カメラ → 実Pico dry-run：A〜E 5ケース、物理駆動なし'),
 ('10/19','PASS','車輪浮上：20 mm/s回転 → 503 ms watchdog → 物理停止・ロック復帰'),
 ('10/20','PASS報告','停止後診断：ACK 0x07・要求左右0・OI Safe=2・drive_enabled=false'),
 ('有人床面','HOLD','単発直進プロファイル準備まで。接続・走行・ロック解除は未実施'),
 ('自律接近','NO-GO','旋回・連続走行・YOLO / Navigationからの実機自律駆動・回収'),
 ('回帰試験','249件成功報告','今回この環境では再実行していない'),
]
for i,(day,status,detail) in enumerate(rows):
    yy=1710+i*50
    txt(100,yy,day,28,blue)
    txt(345,yy,status,28,orange if status in ('HOLD','NO-GO') else blue)
    txt(650,yy,detail,27,gray)

panel(60,2115,2280,310,'今後の優先事項')
rows=[
 ('1','配布コード・UART配線・電源・プロトコル版と、停止診断の証拠を同じ試験IDで固定'),
 ('2','既存チェックリストのHOLD要因を解消。有人単発床面は未実施として管理し、結果を記録'),
 ('3','実機姿勢をmapに統一し認識とdry-runで結合。ArUco実装・校正の工程を確定'),
 ('4','9001再接続、実カメラでのモデル評価、コード・重み・ログの同期で再現性を整える'),
]
for i,(priority,detail) in enumerate(rows):
    yy=2200+i*50
    txt(100,yy,priority,29,blue);txt(165,yy,detail,28,gray)
txt(60,2460,'詳細・参照SHA・評価結果：docs/ARCHITECTURE_STATUS_2026-10-20.md  |  図の更新で走行許可・実機設定は変更しない',26,gray)
im.save(Path(__file__).with_name('architecture_2026-10-20.png'))
