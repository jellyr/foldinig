#include "../defineData.h"
//pget(int x, int y)        : ���͉摜�̎w�肵��x,y���W����s�N�Z���̐F��ǂݏo���i0��1�j
//pset(int x, int y, int c) : �o�͉摜�̎w�肵��x,y���W�ɐF���Z�b�g����ic��0��1�j
//width                     : �摜��
//height                    : �摜����

void get_outline(int width, int height,std::vector<int> pget,std::vector<int> &img_coord ){

int sx, sy;    //�ǐՊJ�n�_
int px, py;    //�ǐՓ_
int vec;       //�����_�t���O
int IsFirst;   // 
bool flg = false;

//�摜����{�����L����f��T��
for(sy=0; sy < height; sy++) {
    for(sx=0; sx < width; sx++) {
        //�L����f���������ꍇ���[�v���甲����
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

//�L����f���������Ă����ꍇ�A�ǐՏ����ɓ���
if( sx < width ) {
	//cout << "start\n";
    px = sx;
    py = sy;
    //pset(px, py, 1);
	pget[py*width+px] = 1;
	img_coord.push_back(px);
	img_coord.push_back(py);
    vec = 2;      //�ŏ��̒����_�������ɃZ�b�g����
    IsFirst = 1;
	int i=0;
    //�ǐՊJ�n�_�ƒǐՓ_���������W�Ȃ�܂ŗ֊s�ǐՏ���
    while( px != sx  ||  py != sy || IsFirst == 1) {
			if(vec == 0){//����𒲍�
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
			}else if(vec == 1){ //���𒲍�
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
			}else if(vec == 2){//�����𒲍�
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
			}else if(vec == 3){//���𒲍�
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
			}else if(vec == 4){ //�E���𒲍�
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
			}else if(vec == 5){ //�E�𒲍�
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
			}else if(vec == 6){//�E��𒲍�
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
			}else if(vec == 7){//��𒲍�
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
