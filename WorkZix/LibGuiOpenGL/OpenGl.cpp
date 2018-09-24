#include <stdio.h>
#include <tchar.h>
#include "OpenGl.h"
#include "glut.h"
#include<iostream>
#include <winbase.h >
using namespace std;


COpenGl* COpenGl::m_pGl = NULL;

std::list<Mat> COpenGl::m_RefPointList;
std::list<Point3d> COpenGl::m_RefPointList_;
std::list<Mat> COpenGl::m_RefPointColor;
std::list<Mat> COpenGl::m_PointsPose;
std::list<Mat> COpenGl::m_PointsList;
std::list<Mat> COpenGl::m_PointsColor;
std::list<Mat> COpenGl::m_LinePose;
std::list<Mat> COpenGl::m_LineList;
std::list<Mat> COpenGl::m_LineColor;
std::list<Point3d> COpenGl::m_SpherePt;
std::list<Mat> COpenGl::m_SpherePose;
std::list<float> COpenGl::m_SphereSizeList;
std::list<Mat> COpenGl::m_SphereColor;
std::list<Mat> COpenGl::m_BreakLinePose;
std::list<Mat> COpenGl::m_BreakLineList;
std::list<Mat> COpenGl::m_BreakLineColor;
float COpenGl::m_dAxisLonth;

double COpenGl::param_gluPerspective_zfar;
stVisualRange *COpenGl::tVisualRange = NULL;

static GLfloat xRot = 0.0f;
static GLfloat yRot = 0.0f;
static GLfloat zRot = 0.0f;

static GLfloat xOffset = 0.0f;
static GLfloat yOffset = 0.0f;
static GLfloat zOffset = 0.0f;

static GLfloat scale = 1.0f;

static int base;

#define OPENGL_MAX_AXIS_VALUDE 100000000.f
#define OPENGL_MIN_AXIS_VALUDE -100000000.f
static double dXmax = OPENGL_MIN_AXIS_VALUDE;
static double dXmin = OPENGL_MAX_AXIS_VALUDE;
static double dYmax = OPENGL_MIN_AXIS_VALUDE;
static double dYmin = OPENGL_MAX_AXIS_VALUDE;

static long nDispType = 0;

CRITICAL_SECTION cs;

void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M, GLfloat N);
COpenGl* COpenGl::GetInstance()
{
	if (m_pGl == NULL)
	{
		m_pGl = new COpenGl;
	}

	InitializeCriticalSection( & cs);

	return m_pGl;
}
void COpenGl::Destory()
{
	if (m_pGl)
	{
		delete m_pGl;
		m_pGl = NULL;
	}
}

COpenGl::COpenGl()
{
	m_bAvaliable = false;
	m_dAxisLonth = 1000.0;
}


COpenGl::~COpenGl(void)
{
	glutDestroyWindow(m_hwnd);

	m_bAvaliable = false;
	glDeleteLists(base,256);						// 从内存中删除256个显示列表
}

void COpenGl::Show(int w,int h)
{
	if (m_bAvaliable == true)
		return;

	m_RefPointList.clear();
	m_RefPointList_.clear();
	m_RefPointColor.clear();
	m_PointsPose.clear();
	m_PointsList.clear();
	m_PointsColor.clear();
	m_LinePose.clear();
	m_LineList.clear();
	m_LineColor.clear();
	m_SpherePt.clear();
	m_SpherePose.clear();
	m_SphereSizeList.clear();
	m_SphereColor.clear();
	m_BreakLinePose.clear();
	m_BreakLineList.clear();
	m_BreakLineColor.clear();

	m_Width = w;
	m_Height = h;

	m_hThread = CreateThread(NULL,0,ThreadProcess,this,0,NULL);

	m_bAvaliable = true;
}

DWORD COpenGl::ThreadProcess(LPVOID lparam)
{
	COpenGl* pGl = (COpenGl*)lparam;

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(pGl->m_Width,pGl->m_Height);
	pGl->m_hwnd = glutCreateWindow("Shiny Jet");
	glutReshapeFunc(ChangeSize);
	glutSpecialFunc(SpecialKeys);
	glutKeyboardFunc(KeyboardKeys);
	glutDisplayFunc(RenderScene);
	glutMouseFunc(MouseKeys);
	glutIdleFunc(Idle);
	SetupRC();
	glutMainLoop();

	return 0;
}

void COpenGl::BuildFont()
{
	HFONT	font;						// 字体句柄
	HFONT	oldfont;						// 旧的字体句柄

	base = glGenLists(96);					// 创建96个显示列表

	font = CreateFont(-24,              //字体高度,为负数时自动更改为默认值
		0,                              // 字体宽度,为0时自动更改为默认值
		0,				                // 字体的旋转角度 Angle Of Escapement
		0,				                // 字体底线的旋转角度Orientation Angle
		FW_BOLD,				        // 字体的重量
		FALSE,				            // 是否使用斜体
		FALSE,				            // 是否使用下划线
		FALSE,				            // 是否使用删除线
		ANSI_CHARSET,			        // 设置字符集
		OUT_TT_PRECIS,			        // 输出精度
		CLIP_DEFAULT_PRECIS,		    // 裁剪精度
		ANTIALIASED_QUALITY,		    // 输出质量
		FF_DONTCARE|DEFAULT_PITCH,		// Family And Pitch
		_T("Courier New"));			    // 字体名称

	oldfont = (HFONT)SelectObject(wglGetCurrentDC(),font);
	wglUseFontBitmaps(wglGetCurrentDC(),32,96,base);
	SelectObject(wglGetCurrentDC(),oldfont);
	DeleteObject(font);
}

void COpenGl::glText(char* fmt, ...)
{
	char		text[256];				// 保存文字串
	va_list		ap;					// 指向一个变量列表的指针

	if (fmt == NULL)						// 如果无输入则返回
		return;

	va_start(ap, fmt);						// 分析可变参数
	vsprintf_s(text,fmt,ap);
	va_end(ap);						// 结束分析

	glPushAttrib(GL_LIST_BIT);					// 把显示列表属性压入属性堆栈
	glListBase(base-32);					// 设置显示列表的基础值

	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);		// 调用显示列表绘制字符串
	glPopAttrib();						// 弹出属性堆栈
}

// Called to draw scene
void COpenGl::RenderScene(void)
{
	// Clear the window with current clearing color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Save the matrix state and do the rotations
	glPushMatrix();
	{
		glRotatef(xRot, 1.0f, 0.0f, 0.0f);
		glRotatef(yRot, 0.0f, 1.0f, 0.0f);
		glRotatef(zRot, 0.0f, 0.0f, 1.0f);

		glTranslatef(xOffset,yOffset,zOffset);

		glScalef(scale,scale,scale);

		DrawAxis();
		EnterCriticalSection( & cs); 
		DrawPoints();
		DrawRefPoints();
		DrawLines();
		DrawBreakLines();
		DrawSphere();
		LeaveCriticalSection( & cs); 
		
	}
	glPopMatrix();	

	glutSwapBuffers();
}

// This function does any needed initialization on the rendering
// context. 
void COpenGl::SetupRC()
{
	BuildFont();

	glEnable(GL_DEPTH_TEST);	// Hidden surface removal
	glFrontFace(GL_CCW);		// Counter clock-wise polygons face out
	glEnable(GL_CULL_FACE);		// Do not calculate inside of jet

	// Light blue background
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f );
}

void COpenGl::KeyboardKeys(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'w':
		//yOffset += 1.0f;
		yOffset += 10.0f;
		break;
	case 's':
		//yOffset -= 1.0f;
		yOffset -= 10.0f;
		break;
	case 'a':
		//xOffset -= 1.0f;
		xOffset -= 10.0f;
		break;
	case 'd':
		//xOffset += 1.0f;
		xOffset += 10.0f;
		break;
	case 'r':
		nDispType = 0;
		AutoSize();
		ResetSize();
		break;
	case 'm':
		nDispType++;
		if (nDispType>3)
			nDispType = 0;
		AutoSize();
		break;
	default:
		break;
	}

	glutPostRedisplay();
}

/////////////////////////////////////////////////////
// Handle arrow keys
void COpenGl::SpecialKeys(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_UP:
		xRot-= 1.0f;
		break;
	case GLUT_KEY_DOWN:
		xRot += 1.0f;
		break;
	case GLUT_KEY_LEFT:
		yRot -= 1.0f;
		break;
	case GLUT_KEY_RIGHT:
		yRot += 1.0f;
		break;
	case GLUT_KEY_HOME:
		zRot -= 1.0f;
		break;
	case GLUT_KEY_END:
		zRot += 1.0f;
		break;
	case GLUT_KEY_PAGE_UP:
		scale *= 1.1f;
		AutoSize();
		break;
	case GLUT_KEY_PAGE_DOWN:
		scale /= 1.1f;
		AutoSize();
		break;
	default:
		break;
	}

//	printf("xRot=%f,yRot=%f,zRot=%f,scale=%f\n",xRot,yRot,zRot,scale);

// 	if (scale<1.0f)
// 	{
// 		scale = 1.0f;
// 	}
// 
// 	if(xRot > 356.0f)
// 		xRot = 0.0f;
// 
// 	if(xRot < -1.0f)
// 		xRot = 355.0f;
// 
// 	if(yRot > 356.0f)
// 		yRot = 0.0f;
// 
// 	if(key < -1.0f)
// 		yRot = 355.0f;

	// Refresh the Window
	glutPostRedisplay();
}

void COpenGl::MouseKeys(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_MIDDLE_BUTTON:
		break;
	default:
		break;
	}
}

void COpenGl::Idle()
{
	glutPostRedisplay();
}

//////////////////////////////////////////////////////////
// Reset projection and light position
void COpenGl::ChangeSize(int w, int h)
{
	GLfloat fAspect;

	// Prevent a divide by zero
	if(h == 0)
		h = 1;

	// Set Viewport to window dimensions
	glViewport(0, 0, w, h);

	// Reset coordinate system
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	fAspect = (GLfloat) w / (GLfloat) h;
//	gluPerspective(45.0f, fAspect, 1.0f, 225.0f);
	//gluPerspective(100.f, fAspect, 1.0f, 10000.f);
	gluPerspective(100.f, fAspect, 1.0f, param_gluPerspective_zfar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(0.0f, 0.0f, -150.0f);
}

void COpenGl::DrawAxis()
{
	glColor3ub(255,0,0);
	glBegin(GL_LINES);
	{
		glVertex3d(0,0,0);
		glVertex3d(m_dAxisLonth,0,0);
	}
	glEnd();

	glRasterPos3f(m_dAxisLonth,0,0);
	glText("x");

	glColor3ub(0,255,0);
	glBegin(GL_LINES);
	{
		glVertex3d(0,0,0);
		glVertex3d(0,m_dAxisLonth,0);
	}
	glEnd();

	glRasterPos3f(0,m_dAxisLonth,0);
	glText("y");

	glColor3ub(0,0,255);
	glBegin(GL_LINES);
	{
		glVertex3d(0,0,0);
		glVertex3d(0,0,m_dAxisLonth);
	}
	glEnd();

	glRasterPos3f(0,0,m_dAxisLonth);
	glText("z");
}

void COpenGl::DrawPoints()
{
	// For show Pts
	glColor3ub(0,255,0);

	glPointSize(1);
	glBegin(GL_POINTS);
	{
		std::list<Mat>::const_iterator its = m_PointsList.begin();
		std::list<Mat>::const_iterator ite = m_PointsList.end();

		std::list<Mat>::const_iterator its_ = m_PointsColor.begin();
		std::list<Mat>::const_iterator ite_ = m_PointsColor.end();

		for (;(its != ite) && (its_ != ite_); its++, its_++)
		{
			if (its_->data)
			{
				glColor3ub(its_->at<uchar>(0,0),its_->at<uchar>(0,1),its_->at<uchar>(0,1));
			}
			else
			{
				glColor3ub(0,255,0);
			}
			for (int i = 0; i < its->rows; i++)
			{
				
				glVertex3d(its->at<double>(i,0),its->at<double>(i,1),its->at<double>(i,2));
			}
		}
	}
	glEnd();
}

void COpenGl::DrawLines()
{
	glLineWidth (1.0);
	std::list<Mat>::const_iterator its = m_LineList.begin();
	std::list<Mat>::const_iterator ite = m_LineList.end();

	std::list<Mat>::const_iterator its_ = m_LineColor.begin();
	std::list<Mat>::const_iterator ite_ = m_LineColor.end();

	for (;(its != ite) && (its_ != ite_);its++, its_++)
	{
		if (its_->data)
		{
			glColor3ub(its_->at<uchar>(0,0),its_->at<uchar>(0,1),its_->at<uchar>(0,2));
		}
		else
		{
			glColor3ub(255,255,255);
		}
		for (int i = 0; i < its->rows - 1; i++)
		{
			glBegin(GL_LINES);
			glVertex3d(its->at<double>(i,0),its->at<double>(i,1),its->at<double>(i,2));
			glVertex3d(its->at<double>(i+1,0),its->at<double>(i+1,1),its->at<double>(i+1,2));
			glEnd();
		}
	}
}

void COpenGl::DrawSphere()
{
	std::list<Point3d>::const_iterator its = m_SpherePt.begin();
	std::list<Point3d>::const_iterator ite = m_SpherePt.end();

	std::list<Mat>::const_iterator its_ = m_SphereColor.begin();
	std::list<Mat>::const_iterator ite_ = m_SphereColor.end();

	std::list<float>::const_iterator its__ = m_SphereSizeList.begin();
	std::list<float>::const_iterator ite__ = m_SphereSizeList.end();

	for (; (its != ite) && (its_ != ite_) && (its__ != ite__); its++, its_++, its__++)
	{
		const Point3d& pt = *its;
		const Mat& clor = *its_; 
		const float& sz = *its__;
		glColor3ub(clor.at<uchar>(0,0),clor.at<uchar>(0,1),clor.at<uchar>(0,2));
		drawSphere(pt.x, pt.y, pt.z,
			sz,15,15);
	}
	
}

void COpenGl::DrawBreakLines()
{
	glLineWidth(1.0);
	glLineStipple (1,0x0303);
	glEnable( GL_LINE_STIPPLE );
	std::list<Mat>::const_iterator its = m_BreakLineList.begin();
	std::list<Mat>::const_iterator ite = m_BreakLineList.end();

	std::list<Mat>::const_iterator its_ = m_BreakLineColor.begin();
	std::list<Mat>::const_iterator ite_ = m_BreakLineColor.end();

	for (;(its != ite) && (its_ != ite_);its++, its_++)
	{
		if (its_->data)
		{
			glColor3ub(its_->at<uchar>(0,0),its_->at<uchar>(0,1),its_->at<uchar>(0,1));
		}
		else
		{
			glColor3ub(255,255,255);
		}
		for (int i = 0; i < its->rows - 1; i++)
		{
			glBegin(GL_LINES);
			glVertex3d(its->at<double>(i,0),its->at<double>(i,1),its->at<double>(i,2));
			glVertex3d(its->at<double>(i+1,0),its->at<double>(i+1,1),its->at<double>(i+1,2));
			glEnd();
		}
	}
	glDisable( GL_LINE_STIPPLE );
}

void COpenGl::DrawRefPoints()
{
	glColor3ub(255,0,0);
	glPointSize(2);
	glBegin(GL_POINTS);
	{
		std::list<Point3d>::const_iterator its = m_RefPointList_.begin();
		std::list<Point3d>::const_iterator ite = m_RefPointList_.end();

		std::list<Mat>::const_iterator its_ = m_RefPointColor.begin();
		std::list<Mat>::const_iterator ite_ = m_RefPointColor.end();

		for (;(its != ite) && (its_ != ite_); its++, its_++)
		{
			if (its_->data)
			{
				glColor3ub(its_->at<uchar>(0,0),its_->at<uchar>(0,1),its_->at<uchar>(0,2));
			}
			else
			{
				glColor3ub(255,0,0);
			}
			glVertex3d(its->x,its->y,its->z);
		}
	}
	glEnd();

	glColor3ub(255,255,255);
	glPointSize(4);
	glBegin(GL_POINTS);
	{
		if (m_RefPointList_.size()>0)
		{
			Point3d& ite = m_RefPointList_.back();
			glVertex3d(ite.x,ite.y,ite.z);
		}
	}

	glEnd();
}

void COpenGl::Clear()
{
	EnterCriticalSection( & cs); 
	m_RefPointList.clear();
	m_RefPointList_.clear();
	m_RefPointColor.clear();
	m_PointsPose.clear();
	m_PointsList.clear();
	m_PointsColor.clear();
	m_LineList.clear();
	m_LinePose.clear();
	m_LineColor.clear();
	m_SpherePose.clear();
	m_SphereSizeList.clear();
	m_SphereColor.clear();
	m_BreakLineList.clear();
	m_BreakLinePose.clear();
	m_BreakLineColor.clear();
	LeaveCriticalSection( & cs); 
}

void COpenGl::ClearRefPoints()
{
	EnterCriticalSection( & cs); 
	m_RefPointList.clear();
	m_RefPointList_.clear();
	m_RefPointColor.clear();
	LeaveCriticalSection( & cs); 
}

void COpenGl::ClearPoints()
{
	EnterCriticalSection( & cs); 
	m_PointsPose.clear();
	m_PointsList.clear();
	m_PointsColor.clear();
	LeaveCriticalSection( & cs); 
}

void COpenGl::ClearLines()
{
	EnterCriticalSection( & cs); 
	m_LineList.clear();
	m_LinePose.clear();
	m_LineColor.clear();
	LeaveCriticalSection( & cs); 
}

void COpenGl::ClearSphere()
{
	EnterCriticalSection( & cs);
	m_SpherePt.clear();
	m_SphereColor.clear();
	m_SpherePose.clear();
	m_SphereSizeList.clear();
	LeaveCriticalSection( & cs); 
}

void COpenGl::ClearBreakLines()
{
	EnterCriticalSection( & cs); 
	m_BreakLineList.clear();
	m_BreakLinePose.clear();
	m_BreakLineColor.clear();
	LeaveCriticalSection( & cs); 
}

void COpenGl::CheckSize()
{
	if (m_RefPointList_.size() == 0)
	{
		dXmax = OPENGL_MIN_AXIS_VALUDE;
		dXmin = OPENGL_MAX_AXIS_VALUDE;
		dYmax = OPENGL_MIN_AXIS_VALUDE;
		dYmin = OPENGL_MAX_AXIS_VALUDE;
	}
	else if (m_RefPointList_.size() == 1)
	{
		dXmax = m_RefPointList_.front().x;
		dXmin = m_RefPointList_.front().x;
		dYmax = m_RefPointList_.front().y;
		dYmin = m_RefPointList_.front().y;
	}
	else
	{
		if (dXmin >= m_RefPointList_.back().x)
		{
			dXmin = m_RefPointList_.back().x;
			//				printf("dXmin = %.9f\n",dXmin);
		}
		if (dXmax <= m_RefPointList_.back().x)
		{
			dXmax = m_RefPointList_.back().x;
			//				printf("dXmax = %.9f\n",dXmax);
		}
		if (dYmin >= m_RefPointList_.back().y)
		{
			dYmin = m_RefPointList_.back().y;
			//				printf("dYmin = %.9f\n",dYmin);
		}
		if (dYmax <= m_RefPointList_.back().y)
		{
			dYmax = m_RefPointList_.back().y;
			//				printf("dYmax = %.9f\n",dYmax);
		}
	}
}

void COpenGl::AutoSize()
{
	CheckSize();

	if (nDispType == 0)//自动缩放，全局视角
	{
		xOffset = 0-(dXmax+dXmin)/2;
		yOffset = 0-(dYmax+dYmin)/2;

		double dDeltaMax = max((dXmax-dXmin),(dYmax-dYmin));
		if (dDeltaMax == 0)
		{
			scale = 1.f;
		}
		else
		{
			scale = 80.f/dDeltaMax;
		}
		yOffset*=scale;
		xOffset*=scale;
	}
	if (nDispType == 1)//自动跟随当前点
	{
		xOffset = 0 - m_RefPointList_.back().x;
		yOffset = 0 - m_RefPointList_.back().y;
		yOffset*=scale;
		xOffset*=scale;
	}
	if (nDispType == 2)//自由模式
	{
		;
	}
	if (nDispType == 3 && m_RefPointList.size()>0)//第一视角模式
	{
		Mat angle;
		Rodrigues(m_RefPointList.back()(Range(0,3),Range(0,3)),angle);
		xOffset = 0-m_RefPointList_.back().x;
		yOffset = 0-m_RefPointList_.back().y;
		yOffset*=scale;
		xOffset*=scale;

		zRot = -1.0 * angle.at<double>(2)/CV_PI*180;
//		printf("xRot=%f,yRot=%f,zRot=%f,scale=%f\n",xRot,yRot,zRot,scale);
	}
}

void COpenGl::ResetSize()
{
	xOffset = 0-(dXmax+dXmin)/2;
	yOffset = 0-(dYmax+dYmin)/2;
	
	double dDeltaMax = max((dXmax-dXmin),(dYmax-dYmin));
	if (dDeltaMax <= 0)
	{
		scale = 1.f;
	}
	else
	{
		scale = 80.f/dDeltaMax;
	}
	yOffset*=scale;
	xOffset*=scale;

	xRot = 0.0f;
	yRot = 0.0f;
	zRot = 0.0f;
}

void COpenGl::Set_gluPerspective_zfar( double dbzfar )
{
	param_gluPerspective_zfar = dbzfar;
}

void COpenGl::Set_VisualRange( stVisualRange *pRange )
{
	tVisualRange = pRange;
}

void COpenGl::AddRefPoints(Mat RefPoint, Mat Color)
{
//	EnterCriticalSection( & cs); 
	m_RefPointList.push_back(RefPoint.clone());
	m_RefPointColor.push_back(Color.clone());
	m_RefPointList_.push_back(Point3d(RefPoint.at<double>(0,3),RefPoint.at<double>(1,3),RefPoint.at<double>(2,3)));
//	LeaveCriticalSection( & cs); 
	AutoSize();
}

void COpenGl::AddRefPoints(double x, double y, double heading, Mat Color/* = Mat()*/)
{
	Mat RefPt = Mat::eye(4,4,CV_64F);
	RefPt.at<double>(0,3) = x;
	RefPt.at<double>(1,3) = y;
	Mat R = (Mat_<double>(1,3) << 0.0, 0.0, heading/180.0*CV_PI);
	Mat R_;
	Rodrigues(R, R_);
	R_.copyTo(RefPt(Range(0,3),Range(0,3)));
	AddRefPoints(RefPt, Color);
}

void COpenGl::AddPointsRelative(Mat Points, Mat Pose, Mat Color)
{
	if (Pose.data == NULL)
	{
		m_PointsList.push_back(Points);
		m_PointsPose.push_back(Pose);
		m_PointsColor.push_back(Color);
	}
	else
	{
		Mat Points_ = Pose * Points.t();
		Mat Points_t = Points_.t();
		AddPointsRelative(Points_t, Mat(), Color);
	}
}

void COpenGl::AddPointsRelative(double x, double y, double heading, Mat Points, Mat Color/* = Mat()*/)
{
	if (Points.data == NULL)
		return;
	Mat Points_;
	if (Points.cols == 3)
	{
		Points_ = Mat::ones(Points.rows, 4 ,Points.type());
		Points.copyTo(Points_(Range::all(), Range(0,3)));
	}
	else
	{
		Points.copyTo(Points_);
	}
	Mat Pose = Mat::eye(4,4,CV_64F);
	Pose.at<double>(0,3) = x;
	Pose.at<double>(1,3) = y;
	Mat R = (Mat_<double>(1,3) << 0.0, 0.0, heading/180.0*CV_PI);
	Mat R_;
	Rodrigues(R, R_);
	R_.copyTo(Pose(Range(0,3),Range(0,3)));
	AddPointsRelative(Points_, Pose, Color);
}

void COpenGl::AddLinesRelative(Mat Lines, Mat Pose, Mat Color)
{
//	EnterCriticalSection( & cs); 
	m_LineList.push_back(Lines);
	m_LinePose.push_back(Pose);
	m_LineColor.push_back(Color);
//	LeaveCriticalSection( & cs); 
}

void COpenGl::AddBreakLinesRelative(Mat Lines, Mat Pose, Mat Color)
{
//	EnterCriticalSection( & cs); 
	m_BreakLineList.push_back(Lines);
	m_BreakLinePose.push_back(Pose);
	m_BreakLineColor.push_back(Color);
//	LeaveCriticalSection( & cs); 
}

void COpenGl::AddSphereRelative(Point3d pt, float sz, Mat Pose, Mat Color)
{
	m_SpherePt.push_back(pt);
	m_SphereSizeList.push_back(sz);
	m_SpherePose.push_back(Pose);
	m_SphereColor.push_back(Color);
}

void COpenGl::SetCameraMode(int nMode)
{
	nDispType = nMode;
}

void COpenGl::SetScale(double dScale)
{
	scale = dScale;
}

void COpenGl::SetAxisLenth(float dLenth)
{
	m_dAxisLonth = dLenth;
}

void drawSphere(GLfloat xx, GLfloat yy, GLfloat zz, GLfloat radius, GLfloat M, GLfloat N)
{
	float step_z = CV_PI/M;
	float step_xy = 2*CV_PI/N;
	float x[4],y[4],z[4];

	float angle_z = 0.0;
	float angle_xy = 0.0;
	int i=0, j=0;
	glBegin(GL_QUADS);
	for(i=0; i<M; i++)
	{
		angle_z = i * step_z;

		for(j=0; j<N; j++)
		{
			angle_xy = j * step_xy;

			x[0] = radius * sin(angle_z) * cos(angle_xy);
			y[0] = radius * sin(angle_z) * sin(angle_xy);
			z[0] = radius * cos(angle_z);

			x[1] = radius * sin(angle_z + step_z) * cos(angle_xy);
			y[1] = radius * sin(angle_z + step_z) * sin(angle_xy);
			z[1] = radius * cos(angle_z + step_z);

			x[2] = radius*sin(angle_z + step_z)*cos(angle_xy + step_xy);
			y[2] = radius*sin(angle_z + step_z)*sin(angle_xy + step_xy);
			z[2] = radius*cos(angle_z + step_z);

			x[3] = radius * sin(angle_z) * cos(angle_xy + step_xy);
			y[3] = radius * sin(angle_z) * sin(angle_xy + step_xy);
			z[3] = radius * cos(angle_z);

			for(int k=0; k<4; k++)
			{
				glVertex3f(xx+x[k], yy+y[k],zz+z[k]);
			}
		}
	}
	glEnd();
}