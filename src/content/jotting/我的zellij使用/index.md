---
title: 锟綀戠殑zellij浣跨敤
date: 2026-02-22
timestamp: 2026-02-22T16:44:14+08:00
slug: zellij
category: jotting
tags:
  - Tools
---

## 1. 浼氳瘽绠＄悊锛堟寔涔呭寲浣犵殑宸ヤ綔锛�

Zellij 鐨勬牳蹇冭兘鍔涘湪浜庡嵆渚� SSH 鏂�紑锛屼綘鐨勪唬鐮佸拰杩涚▼渚濈劧鍦ㄨ繍琛屻€�

- **鍚�姩/鍒涘缓鍛藉悕浼氳瘽锛�** `zellij -s <鍚嶇О>`锛堝� `zellij -s dev`锛夈€�
    
- **鏌ョ湅褰撳墠鎵€鏈変細璇濓細** `zellij ls`銆�
    
- **鏅鸿兘鎭㈠�锛堟渶甯哥敤锛夛細** `zellij a -c`銆�
    
    - _閫昏緫锛歘 鏈夋棫浼氳瘽灏辨仮澶嶏紝娌℃湁灏辨柊寤恒€傚缓璁�妸杩欎釜鍛戒护鍐欐垚鍒�悕锛坅lias锛夈€�
        
- **閫€鍑轰絾涓嶅叧闂�紙Detach锛夛細** `Ctrl + o` 鐒跺悗鎸� `d`銆�
    

## 2. 鐣岄潰鎺у埗锛圱ab 涓� Pane锛�

Zellij 鎶婄粓绔�垎鎴愪簡鈥滄爣绛鹃〉鈥濆拰鈥滅獥鏍尖€濄€�

- **绐楁牸 (Pane) 鎿嶄綔 (`Ctrl + p`)锛�**
    
    - `n`锛氭柊寤虹獥鏍笺€�
        
    - `d`锛氬悜涓嬪垎鍓� / `r`锛氬悜鍙冲垎鍓层€�
        
    - `f`锛氬垏鎹㈡偓娴�/宓屽叆鐘舵€侊紙**Floating**锛岄潪甯搁€傚悎涓存椂璺戜釜鍛戒护锛夈€�
        
    - `z`锛氬叏灞忓垏鎹�紙Toggle Maximize锛夈€�
        
- **鏍囩�椤� (Tab) 鎿嶄綔 (`Ctrl + t`)锛�**
    
    - `n`锛氭柊寤烘爣绛俱€�
        
    - `r`锛氶噸鍛藉悕褰撳墠鏍囩�锛堝尯鍒嗕笉鍚岄」鐩�級銆�
        
- **璋冩暣澶у皬 (`Ctrl + n`)锛�** 杩涘叆鍚庣敤鏂瑰悜閿�皟鏁村綋鍓嶇獥鏍煎昂瀵搞€�
    

## 3. 甯冨眬绯荤粺 (Layouts)

閫氳繃 `.kdl` 鏂囦欢瀹炵幇涓€閿�惎鍔ㄥ�鏉傜殑寮€鍙戠幆澧冿紝涓嶅啀闇€瑕佹墜鍔ㄥ垎灞忋€�

- **瀹氫箟甯冨眬锛�** 鍦� `~/.config/zellij/layouts/` 涓嬪垱寤烘枃浠躲€�
    
- **鏍稿績璇�硶锛�**
    
    浠ｇ爜娈�
    
    ```
    layout {
        pane size=1 borderless=true { plugin location="zellij:tab-bar"; }
        pane split_direction="vertical" {
            pane focus=true // 杩欓噷杩愯�浣犵殑缂栬緫鍣�
            pane size="25%" // 杩欓噷鏀捐皟璇曠粓绔�
        }
        pane size=1 borderless=true { plugin location="zellij:status-bar"; }
    }
    ```
    
- **鍔犺浇甯冨眬锛�** `zellij --layout <鏂囦欢鍚�>`銆�
    

## 4. 鐜颁唬鍖栫殑杩涢樁鍔熻兘

- **绱у噾 UI 妯″紡锛�** 濡傛灉瑙夊緱涓婁笅鐘舵€佹爮澶�崰浣嶏紝浣跨敤 `zellij --layout compact`銆�
    
- **澶氳妭鐐瑰崗浣滐細** 涓や釜浜� SSH 鍒板悓涓€鍙版満鍣�紝`attach` 鍒板悓涓€涓�細璇濓紝鍙�互瀹炵幇鐪熸�鐨勨€滆繙绋嬬粨瀵圭紪绋嬧€濓紝涓斾袱浜哄彲浠ュ悇鑷�湅涓嶅悓鐨� Tab銆�
    
- **蹇�嵎閿�В鑰︼細** 鏃㈢劧浣犻€夋嫨浜嗗湪缂栬緫鍣ㄥ唴绠＄悊鏂囦欢锛屽缓璁�湪 `config.kdl` 涓�厤缃� `Alt + hjkl` 鐩存帴鍒囨崲绐楁牸锛岃烦杩� `Ctrl + p` 杩欎竴姝ュ墠缂€閿�紝浣撻獙浼氶潪甯告帴杩� i3 鎴� Sway銆�
    
## 5. 鍒犻櫎浼氳瘽
#### 涓€閿�垹闄ゆ墍鏈夊凡閫€鍑虹殑浼氳瘽锛�
```shell
zellij ls | grep "EXITED" | awk '{print $1}' | sed 's/\x1b\[[0-9;]*m//g' | xargs -I {} zellij delete-session {}
```

#### 鍒犻櫎鎵€鏈変細璇濓紙鍖呮嫭杩愯�涓�殑锛夛細

```shell
zellij ls | grep -v "Running" | awk '{print $1}' | sed 's/\x1b\[[0-9;]*m//g'| xargs -I {} zellij delete-session {}
```