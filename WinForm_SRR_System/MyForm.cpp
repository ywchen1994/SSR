#include "MyForm.h"

using namespace System;
using namespace WinForm_SRR_System;
[STAThread]
int main(cli::array<System::String^> ^args)
{
	// �إߥ��󱱨���e�A���ҥ� Windows XP ��ı�ƮĪG
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);

	// �إߥD�����ð���
	Application::Run(gcnew MyForm());
	return 0;
}