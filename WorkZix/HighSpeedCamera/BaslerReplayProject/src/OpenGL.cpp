#include "OpenGL.h"

#ifdef WIN32
#include <windows.h>
#include "glut.h"
#else
#include <pthread.h>
#include <algorithm>
#include "GL/freeglut_std.h"
#include "GL/glu.h"
#endif


int COpenGlOpr::m_nDispType = 0;

float COpenGlOpr::m_xRot = 0.f;
float COpenGlOpr::m_yRot = 0.f;
float COpenGlOpr::m_zRot = 0.f;
float COpenGlOpr::m_xOffset = 0.f;
float COpenGlOpr::m_yOffset = 0.f;
float COpenGlOpr::m_zOffset = 0.f;
float COpenGlOpr::m_scale = 1.f;
float COpenGlOpr::m_scalePro = 200.f;

float COpenGlOpr::m_dXmax = FLT_MIN;
float COpenGlOpr::m_dXmin = FLT_MAX;
float COpenGlOpr::m_dYmax = FLT_MIN;
float COpenGlOpr::m_dYmin = FLT_MAX;

std::vector<cv::Point3f> COpenGlOpr::m_refPoints;
std::vector<cv::Scalar> COpenGlOpr::m_refPointsColor;
std::vector<std::vector<cv::Point3f> > COpenGlOpr::m_polyLines;
std::vector<cv::Scalar> COpenGlOpr::m_polyLinesColor;

#ifdef WIN32
unsigned long __stdcall COpenGlOpr::ThreadProcess(void* lparam)
#else
void* COpenGlOpr::ThreadProcess(void* lparam)
#endif
{
	COpenGlOpr* pGl = (COpenGlOpr*)lparam;

    int argc=1;
    char* argv[]={"sample"};
    glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
 	glutInitWindowSize(pGl->m_nWidth, pGl->m_nHeight);
    pGl->m_nHandle = glutCreateWindow("sample");
  	glutReshapeFunc(pGl->ChangeSize);
  	glutSpecialFunc(SpecialKeys);
  	glutKeyboardFunc(KeyboardKeys);
  	glutDisplayFunc(RenderScene);
  	glutMouseFunc(MouseKeys);
  	glutIdleFunc(Idle);
  	SetupRC();
	glutMainLoop();

	return 0;
}

COpenGlOpr::COpenGlOpr()
{
#ifdef WIN32
	HANDLE aaa = CreateThread(NULL,0,ThreadProcess,this,0,NULL);
#else
    pthread_t id;
    int ret = pthread_create(&id, NULL, ThreadProcess, this);
#endif
	m_nHeight = 400;
	m_nWidth = 400;
}

COpenGlOpr::~COpenGlOpr()
{

}

int COpenGlOpr::addRefPoints(const cv::Point3f& point,
		const cv::Scalar& color/* = CV_RGB(255,0,0)*/)
{
	m_refPoints.push_back(point);
	m_refPointsColor.push_back(color);
	AutoSize();

	return 1;
}

int COpenGlOpr::addPolyLine(const std::vector<cv::Point3f>& lines,
	const cv::Scalar& color/* = CV_RGB(255,0,0)*/)
{
	m_polyLines.push_back(lines);
	m_polyLinesColor.push_back(color);

	return 1;
}

void COpenGlOpr::ChangeSize(int w, int h)
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
    gluPerspective(100.f, fAspect, 1.0f, 10000.f);
//	gluPerspective(100.f, fAspect, 1.0f, param_gluPerspective_zfar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(0.0f, 0.0f, -150.0f);
}

void COpenGlOpr::KeyboardKeys(unsigned char key, int x, int y)
{;
 	switch (key)
 	{
 	case 'w':
 		m_yOffset += 10.0f;
 		break;
 	case 's':
 		m_yOffset -= 10.0f;
 		break;
 	case 'a':
 		m_xOffset -= 10.0f;
 		break;
 	case 'd':
 		m_xOffset += 10.0f;
 		break;
 	case 'r':
 		m_nDispType = 0;
 		ResetSize();
 		break;
 	case 'm':
 		m_nDispType++;
 		if (m_nDispType>2)
 			m_nDispType = 0;
 		AutoSize();
 		break;
 	default:
 		break;
 	}
 
 	glutPostRedisplay();
}

/////////////////////////////////////////////////////
// Handle arrow keys
void COpenGlOpr::SpecialKeys(int key, int x, int y)
{
 	switch (key)
 	{
 	case GLUT_KEY_UP:
 		m_xRot-= 1.0f;
 		break;
 	case GLUT_KEY_DOWN:
 		m_xRot += 1.0f;
 		break;
 	case GLUT_KEY_LEFT:
 		m_yRot -= 1.0f;
 		break;
 	case GLUT_KEY_RIGHT:
 		m_yRot += 1.0f;
 		break;
 	case GLUT_KEY_HOME:
 		m_zRot -= 1.0f;
 		break;
 	case GLUT_KEY_END:
 		m_zRot += 1.0f;
 		break;
 	case GLUT_KEY_PAGE_UP:
 		m_scale *= 1.1f;
 		AutoSize();
 		break;
 	case GLUT_KEY_PAGE_DOWN:
 		m_scale /= 1.1f;
 		AutoSize();
 		break;
 	default:
 		break;
 	}
 
 	// Refresh the Window
 	glutPostRedisplay();
}

void COpenGlOpr::MouseKeys(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_MIDDLE_BUTTON:
		break;
	default:
		break;
	}
}

void COpenGlOpr::Idle()
{
	glutPostRedisplay();
}

void COpenGlOpr::RenderScene(void)
{
 	// Clear the window with current clearing color
 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
 	// Save the matrix state and do the rotations
 	glPushMatrix();
 	{
 		glRotatef(m_xRot, 1.0f, 0.0f, 0.0f);
 		glRotatef(m_yRot, 0.0f, 1.0f, 0.0f);
 		glRotatef(m_zRot, 0.0f, 0.0f, 1.0f);
 
 		glTranslatef(m_xOffset,m_yOffset,m_zOffset);
 
 		glScalef(m_scale,m_scale,m_scale);
 
 		DrawAxis();
		DrawRefPoints();
		DrawPolyLines();
//  		EnterCriticalSection( & cs); 
//  		DrawPoints();
//  		DrawRefPoints();
//  		DrawLines();
//  		DrawBreakLines();
//  		DrawSphere();
//  		LeaveCriticalSection( & cs); 
 
 	}
 	glPopMatrix();	
 
 	glutSwapBuffers();
}

void COpenGlOpr::SetupRC()
{ 
 	glEnable(GL_DEPTH_TEST);	// Hidden surface removal
 	glFrontFace(GL_CCW);		// Counter clock-wise polygons face out
 	glEnable(GL_CULL_FACE);		// Do not calculate inside of jet
 
 	// Light blue background
 	glClearColor(0.0f, 0.0f, 0.0f, 1.0f );
}

void COpenGlOpr::CheckSize()
{
	if (m_refPoints.size() == 0)
	{
		m_dXmax = FLT_MIN;
		m_dXmin = FLT_MAX;
		m_dYmax = FLT_MIN;
		m_dYmin = FLT_MAX;
	}
	else if (m_refPoints.size() == 1)
	{
		m_dXmax = m_refPoints.front().x;
		m_dXmin = m_refPoints.front().x;
		m_dYmax = m_refPoints.front().y;
		m_dYmin = m_refPoints.front().y;
	}
	else
	{
		if (m_dXmin >= m_refPoints.back().x)
			m_dXmin = m_refPoints.back().x;
		if (m_dXmax <= m_refPoints.back().x)
			m_dXmax = m_refPoints.back().x;
		if (m_dYmin >= m_refPoints.back().y)
			m_dYmin = m_refPoints.back().y;
		if (m_dYmax <= m_refPoints.back().y)
			m_dYmax = m_refPoints.back().y;
	}
}

void COpenGlOpr::AutoSize()
{
	CheckSize();

	if (m_nDispType == 0)//Auto scale£¬global view
	{
		m_xOffset = 0-(m_dXmax+m_dXmin)/2;
		m_yOffset = 0-(m_dYmax+m_dYmin)/2;

        double dDeltaMax = max((m_dXmax-m_dXmin),(m_dYmax-m_dYmin));
		if (dDeltaMax == 0)
		{
			m_scale = 1.f;
		}
		else
		{
			m_scale = m_scalePro/dDeltaMax;
		}
		m_yOffset*=m_scale;
		m_xOffset*=m_scale;
	}
	if (m_nDispType == 1)//Auto track refpoints
	{
		m_xOffset = 0 - m_refPoints.back().x;
		m_yOffset = 0 - m_refPoints.back().y;
		m_yOffset*=m_scale;
		m_xOffset*=m_scale;
	}
	if (m_nDispType == 2)//free view
	{
		;
	}
}

void COpenGlOpr::ResetSize()
{
	CheckSize();
	m_xOffset = 0-(m_dXmax+m_dXmin)/2;
	m_yOffset = 0-(m_dYmax+m_dYmin)/2;

    double dDeltaMax = max((m_dXmax-m_dXmin),(m_dYmax-m_dYmin));
	if (dDeltaMax <= 0)
	{
		m_scale = 1.f;
	}
	else
	{
		m_scale = m_scalePro/dDeltaMax;
	}
	m_yOffset*=m_scale;
	m_xOffset*=m_scale;

	m_xRot = 0.0f;
	m_yRot = 0.0f;
	m_zRot = 0.0f;
}

void COpenGlOpr::DrawAxis()
{
	glColor3ub(255,0,0);
	glBegin(GL_LINES);
	{
		glVertex3d(0,0,0);
		glVertex3d(1,0,0);
	}
	glEnd();

	glRasterPos3f(1,0,0);
//	glText("x");

	glColor3ub(0,255,0);
	glBegin(GL_LINES);
	{
		glVertex3d(0,0,0);
		glVertex3d(0,1,0);
	}
	glEnd();

	glRasterPos3f(0,1,0);
//	glText("y");

	glColor3ub(0,0,255);
	glBegin(GL_LINES);
	{
		glVertex3d(0,0,0);
		glVertex3d(0,0,1);
	}
	glEnd();

	glRasterPos3f(0,0,1);
//	glText("z");
}

void COpenGlOpr::DrawRefPoints()
{
	// For show Pts
	glColor3ub(0,255,0);

	glPointSize(1);
	glBegin(GL_POINTS);
	{
		for (unsigned int i = 0; i < m_refPoints.size(); i++)
		{
			if (i < m_refPointsColor.size())
			{
				glColor3ub(m_refPointsColor[i][2],m_refPointsColor[i][1],m_refPointsColor[i][0]);
			}
			else
			{
				glColor3ub(0,255,0);
			}
			glVertex3d(m_refPoints[i].x, m_refPoints[i].y, m_refPoints[i].z);
		}
	}
	glEnd();
}

void COpenGlOpr::DrawPolyLines()
{
	glLineWidth (1.0);

	for (unsigned int i = 0; i < m_polyLines.size(); i++)
	{
		if (i < m_polyLinesColor.size())
		{
			glColor3ub(m_polyLinesColor[i][2], m_polyLinesColor[i][1], m_polyLinesColor[i][0]);
		}
		else
		{
			glColor3ub(0,255,0);
		}

		for (unsigned int j = 1; j < m_polyLines[i].size(); j++)
		{
			glBegin(GL_LINES);
			glVertex3d(m_polyLines[i][j-1].x, m_polyLines[i][j-1].y, m_polyLines[i][j-1].z);
			glVertex3d(m_polyLines[i][j].x, m_polyLines[i][j].y, m_polyLines[i][j].z);
			glEnd();
		}
	}
}
