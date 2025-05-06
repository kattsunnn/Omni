# Omni
全方位画像のライブラリ

## GeneratePPI.py
全方位画像から透視投影画像を作成する関数

```python
def GeneratePPI(
        img,
        *,
        eyeDirectionXDeg=None,　
        eyeDirectionYDeg=None,
        eyeDirectionZDeg=None,
        eyeDirectionXCoordinate=None,
        eyeDirectionYCoordinate=None,
        FOVXDeg=None,
        FOVYDeg=None,
        PPISizeH=None,
        PPISizeW=None
        ) -> np.array:
```

### 入力
img：全方位画像（正距円筒画像）

#### 視線方向を角度で指定  
eyeDirectionXDeg：横方向，右に正  
eyeDirectionYDeg：縦方向，右に正  
eyeDirectionZDeg：光軸方向，時計回りに正  

#### 視線方向を画像座標で指定  
eyeDirectionXCoordinate：横方向  
eyeDirectionYCoordinate：縦方向  

#### 視野  
FOVXDeg：横方向の視野  
FOVYDeg：縦方向の視野  

#### 透視投影画像のサイズ  
PPISizeH：縦方向  
PPISizeW：横方向  

視線方向は角度または画像座標で指定します．両方指定することはできません  
視野角または透視投影画像のサイズのいずれか一方が指定されていれば，もう一方の値は自動的に調整され，自然な出力画像となるように設定されます．両方を明示的に指定することも可能です．

### 出力
透視投影画像

