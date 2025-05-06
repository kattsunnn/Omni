import cv2
import numpy as np
import math

# 透視投影画像を生成
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
    
    # 画像のサイズを取得
    imgSize = ImgSize(h=img.shape[0], w=img.shape[1])

    # 視線方向の設定．視線方向の角度XYZ，座標XY，座標XY+角度Zのみ許可
    eyeDirectionisSet = False
    if (eyeDirectionXDeg is not None and eyeDirectionYDeg is not None and eyeDirectionZDeg is not None) and (eyeDirectionXCoordinate is None and eyeDirectionYCoordinate is None):
        eyeDirectionXRad = math.radians(eyeDirectionXDeg)
        eyeDirectionYRad = math.radians(eyeDirectionYDeg)
        eyeDirectionZRad = math.radians(eyeDirectionZDeg)
        eyeDirectionisSet = True
    if (eyeDirectionXDeg is None and eyeDirectionYDeg is None and eyeDirectionZDeg is None) and (eyeDirectionXCoordinate is not None and eyeDirectionYCoordinate is not None):
        eyeDirectionXRad = (eyeDirectionXCoordinate-(imgSize.w/2)) * ((2*math.pi)/imgSize.w)
        eyeDirectionYRad = ((imgSize.h/2)-eyeDirectionYCoordinate) * (math.pi/imgSize.h)
        eyeDirectionZRad = math.radians(0)
        eyeDirectionisSet = True
    if (eyeDirectionXDeg is None and eyeDirectionYDeg is None) and (eyeDirectionZDeg is not None and eyeDirectionXCoordinate is not None and eyeDirectionYCoordinate is not None):
        eyeDirectionXRad = (eyeDirectionXCoordinate-(imgSize.w/2)) * ((2*math.pi)/imgSize.w)
        eyeDirectionYRad = ((imgSize.h/2)-eyeDirectionYCoordinate) * (math.pi/imgSize.h)
        eyeDirectionZRad = math.radians(eyeDirectionZDeg)
        eyeDirectionisSet = True
    if eyeDirectionisSet == False:
        raise ValueError("不正なパラメータ設定です。")

    # 透視投影画像サイズを設定．視野角と画像サイズがどちらか片方，あるいは両方指定されている場合のみ許可
    PPISizeisSet = False
    if (FOVXDeg is not None and FOVYDeg is not None) and (PPISizeW is not None and PPISizeH is not None):
        FOVXRad = math.radians(FOVXDeg)
        FOVYRad = math.radians(FOVYDeg)
        PPISize = ImgSize(h=PPISizeH, w=PPISizeW)
        PPISizeisSet = True
    if (FOVXDeg is not None and FOVYDeg is not None) and (PPISizeW is None and PPISizeH is None):
        FOVXRad = math.radians(FOVXDeg)
        FOVYRad = math.radians(FOVYDeg)
        PPISizeW = int(2 * math.tan( FOVXRad/2 ) * ( imgSize.w / (2*math.pi) ))
        PPISizeH = int(2 * math.tan( FOVYRad/2 ) * ( imgSize.h / math.pi ))
        PPISize = ImgSize(h=PPISizeH, w=PPISizeW)
        PPISizeisSet = True
    if (FOVXDeg is None and FOVYDeg is None) and (PPISizeW is not None and PPISizeH is not None):
        PPISize = ImgSize(h=PPISizeH, w=PPISizeW)
        FOVXRad = 2 * math.atan( (math.pi * PPISize.w) / imgSize.w)
        FOVYRad = 2 * math.atan( (math.pi * PPISize.h) / (2 * imgSize.h))
        PPISizeisSet = True
    if PPISizeisSet == False:
        raise ValueError("不正なパラメータ設定です。視野角または画像サイズを正しく指定してください。")

    #画素間の長さを計算
    pixelLengthX = ( 2 * math.tan(FOVXRad/2) ) / PPISize.w
    pixelLengthY = ( 2 * math.tan(FOVYRad/2) ) / PPISize.h

    # 回転行列の計算
    rotMatY = np.array([[math.cos(eyeDirectionXRad), 0, math.sin(eyeDirectionXRad)],
                         [0, 1, 0],
                         [(-1)*math.sin(eyeDirectionXRad), 0, math.cos(eyeDirectionXRad)]])
    rotMatX = np.array([[1, 0, 0],
                         [0, math.cos(eyeDirectionYRad), (-1)*math.sin(eyeDirectionYRad)],
                         [0, math.sin(eyeDirectionYRad), math.cos(eyeDirectionYRad)]])
    rotMatXY = np.dot(rotMatY, rotMatX)
    unitVecZ = np.array([[0], [0], [1]])
    rotAxisX, rotAxisY, rotAxisZ = np.squeeze( np.dot(rotMatXY, unitVecZ) )
    rotMatZ = np.array([[pow(rotAxisX,2) * (1-math.cos(eyeDirectionZRad)) + math.cos(eyeDirectionZRad), rotAxisX * rotAxisY * (1-math.cos(eyeDirectionZRad)) - (rotAxisZ*math.sin(eyeDirectionZRad)), rotAxisZ * rotAxisX * (1-math.cos(eyeDirectionZRad)) + (rotAxisY*math.sin(eyeDirectionZRad))],
                         [rotAxisX * rotAxisY * (1-math.cos(eyeDirectionZRad)) + rotAxisZ * math.sin(eyeDirectionZRad), pow(rotAxisY, 2) * (1-math.cos(eyeDirectionZRad)) + math.cos(eyeDirectionZRad), rotAxisY * rotAxisZ * (1-math.cos(eyeDirectionZRad)) - (rotAxisX*math.sin(eyeDirectionZRad))],
                         [ rotAxisZ * rotAxisX * (1-math.cos(eyeDirectionZRad)) - (rotAxisY*math.sin(eyeDirectionZRad)), rotAxisY * rotAxisZ * (1-math.cos(eyeDirectionZRad)) + (rotAxisX*math.sin(eyeDirectionZRad)), pow(rotAxisZ, 2) * (1-math.cos(eyeDirectionZRad)) + math.cos(eyeDirectionZRad)]])
    rotMatXYZ = np.dot(rotMatZ, rotMatXY)

    # 透視投影画像の領域を確保
    PPI = np.zeros((PPISize.h, PPISize.w, 3), dtype=np.uint8)
    # 透視投影画像の中心点を求める（W/2, H/2のこと）
    center_u = PPISize.w/2
    center_v = PPISize.h/2
    # 透視投影画像の画素からベクトルを計算．ベクトルから全方位画像の角度を求め，画素を移動する
    for v_p in range(PPISize.h):
        for u_p in range (PPISize.w):
            vec_eye = np.array( [(u_p - center_u)*pixelLengthX, (v_p - center_v)*pixelLengthY, 1] )
            x, y, z =np.dot(rotMatXYZ, vec_eye)
  
            img_theta = math.atan2(x, z)
            img_phi = -math.atan(y/(math.sqrt(x ** 2 + z ** 2)))

            u_e = int( (img_theta+math.pi) * (imgSize.w/(2*math.pi)) )
            v_e = int( ((math.pi/2)-img_phi) * (imgSize.h/math.pi) )

            PPI[v_p][u_p] = img[v_e-1][u_e-1]
        
    return PPI

class ImgSize():
    def __init__(self, h: int, w: int):
        self.h: int = h
        self.w: int = w

    def Check(self):
        print(f"h:{self.h} w:{self.w}")

def main():

    # 画像を読み込み
    imgPath = "Images\camera_1.jpg"
    img = cv2.imread(imgPath)
    # 視線方向を角度で指定
    # eyeDirectionXDeg：横方向，右に正
    # eyeDirectionYDeg：縦方向，右に正
    # eyeDirectionZDeg：光軸方向，時計回りに正
    eyeDirectionXDeg, eyeDirectionYDeg, eyeDirectionZDeg = int(0), int(0), int(0) 

    eyeDirectionXCoordinate, eyeDirectionYCoordinate = int(0), int(0)
    # 視野
    # FOVXDeg：横方向の視野
    # FOVYDeg：縦方向の視野
    FOVXDeg, FOVYDeg = int(90), int(90)
    # 画像サイズ
    PPISizeH = 900
    PPISizeW = 1600
    # 透視投影画像を生成
    PPI = GeneratePPI(
        img,
        eyeDirectionXDeg=eyeDirectionXDeg,
        eyeDirectionYDeg=eyeDirectionYDeg,
        eyeDirectionZDeg=eyeDirectionZDeg,
        # eyeDirectionXCoordinate=eyeDirectionXCoordinate,
        # eyeDirectionYCoordinate=eyeDirectionYCoordinate,
        FOVXDeg=FOVXDeg,
        FOVYDeg=FOVYDeg,
        # PPISizeH=PPISizeH,
        # PPISizeW=PPISizeW
        )
    # 画像をウィンドウで表示
    cv2.imshow("img", PPI)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # 画像の書き出し
    cv2.imwrite('result.jpg', PPI)

# __name__はモジュール名の文字列が格納されている変数．main関数があるときのみmain関数を実行する
if __name__ == "__main__":
    main()