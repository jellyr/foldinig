#include "../defineData.h"
//pget(int x, int y)        : 入力画像の指定したx,y座標からピクセルの色を読み出す（0か1）
//pset(int x, int y, int c) : 出力画像の指定したx,y座標に色をセットする（cは0か1）
//width                     : 画像幅
//height                    : 画像高さ

void get_outline(int width, int height,std::vector<int> pget,std::vector<int> &img_coord ){

int sx, sy;    //追跡開始点
int px, py;    //追跡点
int vec;       //調査点フラグ
int IsFirst;   // 
bool flg = false;

//画像内を捜査し有効画素を探す
for(sy=0; sy < height; sy++) {
    for(sx=0; sx < width; sx++) {
        //有効画素があった場合ループから抜ける
        if( pget[sy*width + sx] != 0 ) {
			//cout << "aaaa\n";
			flg = true;
            break;
        }
    }
	if(flg){
		break;
	}
}

//有効画素が見つかっていた場合、追跡処理に入る
if( sx < width ) {
	//cout << "start\n";
    px = sx;
    py = sy;
    //pset(px, py, 1);
	pget[py*width+px] = 1;
	img_coord.push_back(px);
	img_coord.push_back(py);
    vec = 2;      //最初の調査点を左下にセットする
    IsFirst = 1;
	int i=0;
    //追跡開始点と追跡点が同じ座標なるまで輪郭追跡処理
    while( px != sx  ||  py != sy || IsFirst == 1) {
			if(vec == 0){//左上を調査
                if( pget[(px-1) + (py-1)*width] != 0 ) {
					//cout << "0 ";
                    pget[(px-1) + (py-1)*width] = 1;
                    px = px-1; py = py-1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    vec = 6;
                   // break;
                }else{

					vec = 1;
				}
			}else if(vec == 1){ //左を調査
                if( pget[(px-1) + py*width] != 0 ) {
					//cout << "1 ";
                    pget[(px-1) + py*width] = 1;
                    px = px-1;
					img_coord.push_back(px);
					img_coord.push_back(py);
					//cout << px << " " << py << " ";
                    vec = 0;
                   // break;
                }else {
					vec = 2;
				}
			}else if(vec == 2){//左下を調査
                if( pget[(px-1) +  (py+1)*width] != 0 ) {
					pget[(px-1) +  (py+1)*width] =  1;
                    px = px-1; py = py+1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    IsFirst = 0;
                    vec = 0;
                    //break;
                }else{
					vec = 3;
				}
				//cout << "vec2 is 0\n";
			}else if(vec == 3){//下を調査
                if( pget[px+  (py+1)*width] != 0 ) {
                    pget[px +  (py+1)*width] = 1;
                    py = py+1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    IsFirst = 0;
                    vec = 2;
                  //  break;
                }else{
					vec = 4;
				}
			}else if(vec == 4){ //右下を調査
                if( pget[(px+1) + (py+1)*width] != 0 ) {
                    pget[(px+1) + (py+1)*width] = 1;
                    px = px+1; py = py+1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    IsFirst = 0;
                    vec = 2;
                   // break;
                }else{
					vec = 5;
				}
			}else if(vec == 5){ //右を調査
                if( pget[(px+1) + py*width] != 0 ) {
                     pget[(px+1) + py*width] = 1;
                    px = px+1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    IsFirst = 0;
                    vec = 4;
                   // break;
                }else {
                    vec = 6;
                }
			}else if(vec == 6){//右上を調査
                if( pget[(px+1) +  (py-1)*width] != 0 ) {
                    pget[(px+1) +  (py-1)*width] = 1;
                    px = px+1; py = py-1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    vec = 4;
                  //  break;
                }else{
					vec = 7;
				}
			}else if(vec == 7){//上を調査
                if( pget[px + (py-1)*width] != 0 ) {
                    pget[px + (py-1)*width] = 1;
                    px = px; py = py-1;
					img_coord.push_back(px);
					img_coord.push_back(py);
                    vec = 6;
                   // break;
                }else{
					vec = 0; 
				}
			}
			// << vec << " ";
			//cout << "i: " << i << " " <<  px << " " << py << " " << IsFirst << "\n";
        }

    }

}
