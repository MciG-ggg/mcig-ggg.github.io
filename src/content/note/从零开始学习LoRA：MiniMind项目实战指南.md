---
title: 锟交庨浂寮€濮嬪�涔燣oRA锛歁iniMind椤圭洰瀹炴垬鎸囧崡
date: 2026-02-22
timestamp: 2026-02-22T16:50:09+08:00
slug: 闆跺紑濮嬪�涔爈oraminimind椤圭洰瀹炴垬鎸囧崡
category: note
tags:
  - AI
  - LLM
---

# 浠庨浂寮€濮嬪�涔燣oRA锛歁iniMind椤圭洰瀹炴垬鎸囧崡

## 鐞嗚�鍩虹�

### 浠€涔堟槸LoRA锛�

LoRA锛圠ow-Rank Adaptation锛夋槸涓€绉嶉珮鏁堢殑鍙傛暟寰�皟鏂规硶锛岀敱寰�蒋鍦�2021骞存彁鍑恒€傚畠鐨勬牳蹇冩€濇兂鏄�細**鍐荤粨棰勮�缁冩ā鍨嬫潈閲嶏紝鍙��缁冨皯閲忎綆绉╃煩闃垫潵閫傞厤鏂颁换鍔�**銆�

鎯宠薄涓€涓嬶紝浣犺�璁╀竴涓�細寮归挗鐞寸殑浜哄幓瀛︽媺灏忔彁鐞淬€備紶缁熺殑鍋氭硶鏄�噸鏂版暀浠栨墍鏈夋妧宸э紙鍏ㄥ弬鏁板井璋冿級锛岃€孡oRA鐨勫仛娉曟槸锛氫繚鎸佷粬鍘熸湁鐨勯挗鐞存妧鑹轰笉鍙橈紝鍙�暀浠栧嚑涓�壒娈婄殑鎸囨硶鎶€宸э紙浣庣З閫傞厤锛夈€�

### 涓轰粈涔堥€夋嫨LoRA锛�

1. **鍙傛暟閲忔瀬灏�**锛氬�浜庝竴涓�40B妯″瀷锛宺ank=8鏃禠oRA鍙傛暟鍙�崰0.1-1%
2. **璁�粌閫熷害蹇�**锛氬彧闇€浼樺寲灏戦噺浣庣З鍙傛暟
3. **鍐呭瓨鏁堢巼楂�**锛氭棤闇€瀛樺偍瀹屾暣鐨勬ā鍨嬪壇鏈�
4. **鍙�粍鍚堟€у己**锛氬彲浠ュ湪鍚屼竴涓�ā鍨嬩笂鍔犺浇澶氫釜LoRA閫傞厤鍣�

## 鏍稿績鏋舵瀯

### LoRA缃戠粶缁撴瀯

璁╂垜鍏堜粠鏈€鏍稿績鐨凩oRA绫诲紑濮嬪垎鏋愶細

```python
class LoRA(nn.Module):
    def __init__(self, in_features, out_features, rank):
        self.rank = rank  # 浣庣З绉╁€硷紙閫氬父寰堝皬锛屽�4-16锛�
        self.A = nn.Linear(in_features, rank, bias=False)   # 闄嶇З鐭╅樀 r脳d
        self.B = nn.Linear(rank, out_features, bias=False)  # 鍗囩З鐭╅樀 d'脳r
```

**鍏抽敭璁捐�鐐癸細**

- **浣庣З鍒嗚В**锛氬師濮嬫潈閲嶇煩闃� W锛坉脳d'锛夎�鍒嗚В涓� W + BA锛屽叾涓� BA 鐨勭З涓� r锛坮 << d锛�
  - 鏁板�鍘熺悊锛氫换浣曠煩闃甸兘鍙�互鍒嗚В涓轰綆绉╃煩闃电殑涔樼Н
  - 瀹為檯鏁堟灉锛氱敤灏戦噺鐨勫弬鏁板氨鑳借〃绀哄師濮嬫潈閲嶇殑"灏忓箙搴﹁皟鏁�"

- **鍒濆�鍖栫瓥鐣�**锛氱煩闃礎浣跨敤楂樻柉鍒濆�鍖栵紙std=0.02锛夛紝鐭╅樀B鍒濆�鍖栦负闆�
  - A鐭╅樀锛氶珮鏂�垎甯冩彁渚涜壇濂界殑姊�害娴佸姩
  - B鐭╅樀锛氶浂鍒濆�鍖栫‘淇濊�缁冨垵鏈�=W锛岄伩鍏嶇獊鐒舵敼鍙�

- **鏃犲亸缃�**锛氫袱涓�煩闃甸兘浣跨敤bias=False锛岄伩鍏嶅弬鏁板啑浣�

**涓轰粈涔堥€夋嫨浣庣З锛�**
鎯宠薄涓€涓嬩綘瑕佽皟鏁翠竴寮犵収鐗囩殑浜�害锛�
- 鍏ㄥ弬鏁板井璋冿細閲嶆柊璋冩暣姣忎釜鍍忕礌锛堝お鏄傝吹锛�
- LoRA锛氬彧璋冩暣涓€涓�"浜�害婊戝潡"锛堟垚鏈�綆锛屾晥鏋滃ソ锛�

rank鍊奸€夋嫨鍙傝€冿細
- rank=4-8锛氬皬妯″瀷锛岄€傚悎绠€鍗曚换鍔�
- rank=8-16锛氫腑绛夋ā鍨嬶紝閫氱敤鎬у己
- rank>16锛氬ぇ妯″瀷锛屾帴杩戝叏寰�皟鏁堟灉

### Module name鐨勬潵婧�

杩欐槸鎴戝�涔犺繃绋嬩腑閬囧埌鐨勪竴涓�毦鐐癸細褰撲綘璋冪敤`model.named_modules()`鏃讹紝PyTorch浼氶€掑綊鍦伴亶鍘嗘ā鍨嬩腑瀹氫箟鐨勬墍鏈夊瓙妯″潡锛屽苟鏍规嵁浣犲湪`__init__`涓�粰瀹冧滑璧风殑鍙橀噺鍚嶇敓鎴愪竴鏉¤矾寰勩€�

涓句釜绠€鍗曠殑渚嬪瓙锛�
```python
class MyModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.transformer = TransformerBlock() # 鍙橀噺鍚嶆槸 transformer

class TransformerBlock(nn.Module):
    def __init__(self):
        super().__init__()
        self.q_proj = nn.Linear(512, 512) # 鍙橀噺鍚嶆槸 q_proj
```

褰撲綘杩愯�`apply_lora(model)`鍚庯紝`q_proj`鍐呴儴琚�寕杞戒簡涓€涓�悕涓篳lora`鐨勫瓙妯″潡銆傛�鏃讹紝`named_modules()`杩斿洖鐨刵ame灏变細鏄�細

```
transformer
transformer.q_proj
transformer.q_proj.lora锛堣繖灏辨槸浣犺�鎵剧殑鍚嶅瓧锛侊級
```


### 鍙傛暟鍖栨敼閫�

```python
def apply_lora(model, rank=8):
    for name, module in model.named_modules():
        if isinstance(module, nn.Linear) and module.weight.shape[0] == module.weight.shape[1]:
            # 鍙�负鏂瑰舰鏉冮噸鐭╅樀娣诲姞LoRA
            lora = LoRA(module.weight.shape[0], module.weight.shape[1], rank=rank)
            setattr(module, "lora", lora)
            # 淇�敼forward鍑芥暟
            original_forward = module.forward
            def new_forward(x, original_forward=original_forward, lora=lora):
                return original_forward(x) + lora(x)
            module.forward = new_forward
```

**鏀归€犵瓥鐣ワ細**

- 鍙��鐞嗘柟褰㈢煩闃碉紙transformer涓�殑娉ㄦ剰鍔涘眰閫氬父鏄�柟闃碉級
- 鍦ㄦ瘡涓�嚎鎬у眰鍐呴儴娣诲姞LoRA妯″潡
- 閫氳繃monkey patching淇�敼forward鍑芥暟锛歚output = original_forward(x) + lora(x)`

## 璁�粌娴佺▼

### 璁�粌閰嶇疆

```python
# 鍏抽敭瓒呭弬鏁�
args.learning_rate = 1e-4      # 杈冧綆鐨勫�涔犵巼锛岄伩鍏嶇牬鍧忛�璁�粌鏉冮噸
args.epochs = 50               # 璁�粌杞�暟
args.batch_size = 32           # 鎵规�澶у皬
args.rank = 8                  # LoRA绉╁€�
```

### 鍙傛暟鍐荤粨绛栫暐

```python
# 鍐荤粨闈濴oRA鍙傛暟锛屽彧璁�粌LoRA鍙傛暟
for name, param in model.named_parameters():
    if 'lora' in name:
        param.requires_grad = True
        lora_params.append(param)
    else:
        param.requires_grad = False
```

**浼樺娍锛�**

- **鍙傛暟閲忔瀬灏�**锛氬�浜庝竴涓�40B妯″瀷锛宺ank=8鏃禠oRA鍙傛暟鍙�崰0.1-1%
  - 涓句緥锛欱ERT-large (355M鍙傛暟)锛孡oRA rank=8鏃跺彧闇€绾�1.4M鍙傛暟
  - 瀛樺偍绌洪棿锛氫竴涓�40B妯″瀷锛孡oRA鏉冮噸鍙�兘鍙�湁鍑犵櫨MB

- **璁�粌閫熷害蹇�**锛氬彧闇€浼樺寲灏戦噺浣庣З鍙傛暟
  - 璁�粌鏃堕棿锛氶€氬父姣斿叏寰�皟蹇�5-10鍊�
  - 鏀舵暃閫熷害锛歀oRA鍙傛暟灏戯紝鏇村�鏄撴敹鏁�

- **鍐呭瓨鏁堢巼楂�**锛氭棤闇€瀛樺偍瀹屾暣鐨勬ā鍨嬪壇鏈�
  - 鎺ㄧ悊鏃讹細涓€涓�ā鍨� + 澶氫釜灏廘oRA鏂囦欢
  - 閮ㄧ讲鏃讹細鍙�互鍔ㄦ€佸垏鎹�笉鍚岀殑LoRA閫傞厤鍣�

### 鏉冮噸绠＄悊

#### LoRA鏉冮噸淇濆瓨

```python
def save_lora(model, path):
    state_dict = {}
    for name, module in model.named_modules():
        if hasattr(module, 'lora'):
            lora_state = {f'{name}.lora.{k}': v for k, v in module.lora.state_dict().items()}
            state_dict.update(lora_state)
    torch.save(state_dict, path)
```

#### LoRA鏉冮噸鍔犺浇

```python
def load_lora(model, path):
    state_dict = torch.load(path, map_location=model.device)
    for name, module in model.named_modules():
        if hasattr(module, 'lora'):
            lora_state = {k.replace(f'{name}.lora.', ''): v for k, v in state_dict.items()
                         if f'{name}.lora.' in k}
            module.lora.load_state_dict(lora_state)
```

#### 鏂囦欢缁撴瀯锛�

涓绘ā鍨嬶細`./checkpoints/model_name_hidden_size.pth`
LoRA鏉冮噸锛歚./out/lora/lora_name_hidden_size.pth`

## 鎺ㄧ悊閮ㄧ讲

### 鍔ㄦ€佸姞杞絃oRA

```python
if args.lora_weight != 'None':
    apply_lora(model)              # 搴旂敤LoRA缁撴瀯
    load_lora(model, lora_path)    # 鍔犺浇LoRA鏉冮噸
```

### 瀹炴椂鍒囨崲鑳藉姏

鍙�互鍦ㄥ悓涓€涓�富妯″瀷涓婂姞杞戒笉鍚岀殑LoRA鏉冮噸
- 鏀�寔闆舵牱鏈�彁绀猴紙灏戞牱鏈��涔狅級
- 姣忎釜LoRA鏂囦欢浠ｈ〃涓€涓�壒瀹氫换鍔＄殑閫傞厤

**瀹為檯搴旂敤鍦烘櫙锛�**
姣斿�浣犲彲浠ユ湁涓€涓�熀纭€鐨凪iniMind妯″瀷锛岀劧鍚庨拡瀵逛笉鍚岀殑浠诲姟璁�粌涓嶅悓鐨凩oRA閫傞厤鍣�細
- `lora_translation_768.pth` - 涓�枃缈昏瘧浠诲姟
- `lora_qa_768.pth` - 闂�瓟浠诲姟
- `lora_summary_768.pth` - 鏂囨湰鎬荤粨浠诲姟

鎺ㄧ悊鏃跺彧闇€瑕佸姞杞藉�搴旂殑LoRA鏂囦欢鍗冲彲锛屾棤闇€閲嶆柊鍔犺浇鏁翠釜妯″瀷锛岃繖澶уぇ鑺傜渷浜嗗唴瀛樺拰鎺ㄧ悊鏃堕棿銆�