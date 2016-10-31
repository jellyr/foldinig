#pragma once
#ifndef INCLUDE_FOLDING_METHOD
#define INCLUDE_FOLDING_METHOD
#include "defineData.h"
#include "InputFile/InputFile.h"

class COpenGL
{

public:
	//	GLData *data;
	GLData *TDdata;
	bool start;
	double modelScale;
	int  w;
	int h;

	int outline_n;
	int point_n;
	int line_n;
	int face_n;
	int Top_point_n;

	int *part1;
	int *part2;

	int Model_before;

	bool refe_flg;
	bool refe_render_flg;

	double diffx, diffy, diffz;
	
	double model_moveX,model_moveY;

	bool bunny_visible;


	double size;
	double c_x,c_y,c_z;

	bool STL;

	double rotateAngleV_deg;
	double rotateAngleH_deg;
	int preMousePositionX;   // �}�E�X�J�[�\���̈ʒu���L�����Ă����ϐ�
	int preMousePositionY;   // �}�E�X�J�[�\���̈ʒu���L�����Ă����ϐ�
	bool mouseLeftPressed;
	bool mouseRightPressed;

	double border;

	int now_n;

public:
	
	/*�`��p�̊�{�@�\*/
	//	void paintWindow(void);

	/*�������p*/
	void setParam();
	void init();

	//	void modelmotion(int x,  int y);//�}�E�X�ŃI�u�W�F�N�g����]������֐�	

	/*�t�@�C�����o�͊֘A*/
	//	Model* readSTL(std::string filename, bool IsNormalize);//STL�t�@�C����ǂݍ���
	Model* readOBJ(std::string filename,bool IsNormalize);//obj�t�@�C����ǂݍ���
	void InputPolygonalLine(std::string filename, GLData *data);//�܂肽���ݐ���ǂݍ���


	//Model* COpenGL::optimization();//�܂肽���݂̍œK��

	void Trim(GLData *data);
	void Trim(Model *m);
	void renderParts(int i, Model *m, bool flg, Vec3 move, Vec3 scale);
	void setPartsPerspective();
	//	void renderref();

	void optimization(Model *m);
	void Step(double *output, double gap, std::vector<Vec2> points, int size, Vec2 *pointsN, bool One);
	bool optimization_oen_outline(Model *m, int outline_num);
	void optimization_one(Model *m);
	void optimization_color(Model *m);

	void AnimationSet(foldmethod *fold);

	void setPartlinePoint(foldmethod *fold,int state);
	/*void pick3D(int x,int y, int flg , Model *m);
	void pick3D_render(Model *m);
	void pick3D_render_line(Model *m);*/
	void move_point(int line_num, int point_num, foldmethod *fold, double x,double y);

	void outputFolding(Model *m);
	/*void pick3D_render_face(Model *m, std::vector<Vec3> &FaceData);

	void pick3D_renderParts(Model *m, bool flg, Vec3 move, Vec3 scale);
	void pick3D_render_all();
*/
	void CrossSection(Model *m);
	void CrossSectionBottom(Model *m);
	void CrossSection(Vec3 P, Vec3 N, Vec3 Top , Vec3 Bottom, Vec3 dir, std::vector<Vec3> &plane_, Model *m);
	void Quickhull(const std::vector<Vec2> points, Model *m);
	void Top_optimize(Model *m);
	void CutModel(Model *m);
	void AutoOptimization(Model *m);
	void ChangeTop(Model *m,int x, int y);

	void SetPlane(Model *m);
	void CalculateVolume(Model *m);
	void outputObj();//	triangualation����obj�ɂ��ďo��
	void convertFoldingToMesh(Model *m);
	void changeVertexPos(Model *m);
	
	//�f�ʐ���

public:
	void setPerspective( );
	void renderScene(void);

};
#endif

