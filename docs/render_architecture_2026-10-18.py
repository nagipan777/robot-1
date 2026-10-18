"""Architecture snapshot grounded in the October 15 morning report."""
from pathlib import Path
import argparse
import math
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser()
p.add_argument('--font', required=True)
a = p.parse_args()
im = Image.new('RGB', (2400, 2370), '#f5f8fc')
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

txt(60,30,'PAI 最終課題｜10/18の結果を反映したアーキテクチャ',48)
txt(60,100,'2026-10-18 20:23 JST · 本日の作業報告 + GitHub確認。新しいdry-runコードはGitHub未収録。',28,gray)
line([(70,170),(145,170)]);txt(165,150,'本日確認：実カメラ → 実Picoのdry-run',27)
line([(1110,170),(1185,170)],True);txt(1205,150,'OI駆動・物理停止への接続と検証は別工程',27,orange)

panel(60,235,2280,355,'認識・判断｜Mac / Colima・ROS 2（本日の報告経路）')
box(100,320,510,175,'C270 → YOLO → HTTP',
    ['/webcam/detections','実カメラで対象検出を入力','conf 0.30の条件で試験'])
box(705,320,700,175,'床面投影・候補から目標を選択',
    ['/litter/target_candidate','→ /litter/target','領域制限・近接対象の拒否'])
box(1500,320,800,175,'共通Navigation',
    ['姿勢は手動設定 (1.50, 0, 0)','/nav/cmd_vel_unstamped','実測姿勢の自動入力は未統合'])
arrow([(610,405),(705,405)])
arrow([(1405,405),(1500,405)])
txt(100,530,'実カメラ使用 ≠ 実機姿勢の自動推定。今回の姿勢入力は手動のまま。',28,gray)

panel(60,670,2280,360,'本日合格した主経路｜出力切替 dry_run → TCP 9001 → 実Pico W')
box(100,755,510,175,'出力切替：dry_run',
    ['/roomba/cmd_vel_dry_run','UART・OIは使わない','実機走行指令には接続しない'])
box(705,755,700,175,'pico_dry_run_bridge / transport',
    ['protocol v2：session ID・CRC','READY / CMD / ACK','バックオフ・再接続ラッチ'])
box(1500,755,800,175,'実Pico W dry-run receiver',
    ['TCP 9001で指令受信・ACK','再接続後の古い非ゼロ指令を拒否','STOPPED状態を維持'])
arrow([(1900,495),(1900,630),(355,630),(355,755)])
arrow([(610,840),(705,840)])
arrow([(1405,840),(1500,840)])
txt(100,970,'e2e_dry_run_recorder：入力・判定・送信・応答・STOPを記録。全5ケース合格との報告。',28,blue)

panel(60,1110,2280,320,'本日の試験結果｜成功した条件と、主張できる範囲')
rows=[
 ('A / B','遠方マスク合格 ／ 50 cm対象の拒否に合格'),
 ('C','Roomba誤検出0件：conf閾値0.30の試験条件に限定'),
 ('D / E','対象除去から約55 msでSTOP記録 ／ 再接続後の旧指令拒否・STOPPED維持'),
 ('テスト','protocol関連25件成功 ／ 別途31テストは未実施との報告'),
]
for i,(label,detail) in enumerate(rows):
 yy=1190+i*50
 txt(100,yy,label,29,blue)
 txt(410,yy+2,detail,28,gray)

panel(60,1510,2280,325,'別の検証経路｜TCP 9000 / UART / Roomba OI（今日のE2Eでは未使用）')
box(100,1595,510,155,'roomba_bridge_tcp',
    ['走行許可・Safe確認・ARM','短距離の実機統合は別検証'],True)
box(705,1595,700,155,'pico_relay → UART',
    ['本日：501 ms STOP書込・旧世代拒否','車輪回転なし／非ゼロ転送記録なし'],True)
box(1500,1595,800,155,'Roomba 680',
    ['動いている車輪の停止・停止維持は未確認','過去の115200 bps応答・駆動実績とは区別'],True)
arrow([(610,1670),(705,1670)],True)
arrow([(1405,1670),(1500,1670)],True)
txt(100,1780,'TCP 9001の合格をTCP 9000の物理停止合格へ転用しない。約55 msは車輪の制動時間ではない。',27,orange)

panel(60,1915,2280,305,'今後の課題｜停止確認・通信安定化・姿勢実測へ')
priorities=[
 ('P0','電池・通信条件・配布版を確認し、OI経路で非ゼロ転送 → watchdog → 物理停止保持を検証'),
 ('P1','ACK一時timeout・再接続最大約8秒を切り分ける。待機中もSTOPPED・自動再駆動なしを確認'),
 ('P1','手動姿勢をmap座標の実測姿勢へ変更。本日のコード・5ケースの証拠をGitHubへ同期'),
 ('P2','実測姿勢でdry-run再合格 → 固定目標の短距離走行 → ゴミ目標 → 回収。認識の反復評価も継続'),
]
for i,(label,detail) in enumerate(priorities):
 yy=1995+i*48
 txt(100,yy,label,29,orange)
 txt(230,yy+2,detail,26,gray)
txt(70,2260,'現在地：認識から実PicoまでのE2E dry-runは合格。Roombaの自律走行・回収は未完了。',30,blue)
txt(70,2310,'根拠：10/18 13:38〜13:45・19:52・20:18頃の作業報告。GitHub main 2e04b7e / feature 1a9c903。',25,gray)
out=Path(__file__).with_name('architecture_2026-10-18.png')
im.save(out)
print(out.resolve())
