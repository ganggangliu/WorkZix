// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� PROJECTORMAPPINGOPR_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// PROJECTORMAPPINGOPR_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
#ifdef PROJECTORMAPPINGOPR_EXPORTS
#define PROJECTORMAPPINGOPR_API __declspec(dllexport)
#else
#define PROJECTORMAPPINGOPR_API __declspec(dllimport)
#endif

// �����Ǵ� ProjectorMappingOpr.dll ������
class PROJECTORMAPPINGOPR_API CProjectorMappingOpr {
public:
	CProjectorMappingOpr(void);
	// TODO: �ڴ�������ķ�����
};

extern PROJECTORMAPPINGOPR_API int nProjectorMappingOpr;

PROJECTORMAPPINGOPR_API int fnProjectorMappingOpr(void);
