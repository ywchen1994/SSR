#pragma once
#define _USE_MATH_DEFINES
#include <Windows.h>
#include"math.h"
#include<vector>
typedef unsigned int uint;
const int TIME_periodic = 13;
const int EVENT_TYPE = TIME_PERIODIC;
namespace WinForm_SRR_System {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO;
	using namespace System::IO::Ports;

	using namespace System::Runtime::InteropServices;
	using namespace std;
#pragma region Timer
	delegate void TimerEventHandler(int id, int msg, IntPtr user, int dw1, int dw2);
	delegate void TestEventHandler(int tick, TimeSpan span);
	[DllImport("winmm.dll")]
	extern "C" int timeSetEvent(int delay, int resolution, TimerEventHandler ^handler, IntPtr user, int eventType);
	[DllImport("winmm.dll")]
	extern "C" int timeKillEvent(int id);
	[DllImport("winmm.dll")]
	extern "C" int timeBeginPeriod(int msec);
	[DllImport("winmm.dll")]
	extern "C" int timeEndPeriod(int msec);
#pragma endregion

	
	int LiDAR_Data[722];
	int Current_Speed = 0;
	struct RadarData
	{
		double X;
		double Y;
		double Velocity;
		double theta;
		int ID;
		int Mod;
	};
	struct Pt
	{
		double x;
		double y;
	};
	vector<Pt> LIDAR_cooridate;
	RadarData R_Data;

	/// <summary>
	/// MyForm 的摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此加入建構函式程式碼
			//
			ComPortRefresh();
			comboBox1->Text = "COM11";
			comboBox2->Text = "COM3";
			comboBox3->Text = "COM10";
			timer1->Interval = 10;
			timer1->Start();
		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  chart1;
	protected:
	private: System::Windows::Forms::ComboBox^  comboBox1;
	private: System::Windows::Forms::Button^  Btn_LiDAR_Connected;
	private: System::IO::Ports::SerialPort^  serialPort_LiDAR;
	private: System::ComponentModel::IContainer^  components;

	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>
		int counter = 0;
		uint format = 25;
		int mTimerId;
		int mTestTick;
		DateTime mTestStart;
		TimerEventHandler ^mHandler;
		bool getLiDARData = false;
	private:bool GetHeader = false;
	private: System::Windows::Forms::Timer^  timer1;
	private: System::Windows::Forms::Button^  Btn_LiDAR_DisConnect;
	private: System::Windows::Forms::ComboBox^  comboBox2;
	private: System::Windows::Forms::Button^  Btn_Tbox_Connect;
	private: System::IO::Ports::SerialPort^  serialPort_Tbox;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  Txt_CarSpeed;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::ComboBox^  comboBox3;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Button^  Btn_Radar_Connect;
	private: System::IO::Ports::SerialPort^  serialPort_Radar;
	private: System::Windows::Forms::Button^  Btn_Refresh_Combox;
	private: System::Windows::Forms::Label^  label5;


#pragma region Windows Form Designer generated code
			 /// <summary>
			 /// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
			 /// 這個方法的內容。
			 /// </summary>
			 void InitializeComponent(void)
			 {
				 this->components = (gcnew System::ComponentModel::Container());
				 System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
				 System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 System::Windows::Forms::DataVisualization::Charting::Series^  series2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
				 this->chart1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
				 this->comboBox1 = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_LiDAR_Connected = (gcnew System::Windows::Forms::Button());
				 this->serialPort_LiDAR = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
				 this->Btn_LiDAR_DisConnect = (gcnew System::Windows::Forms::Button());
				 this->comboBox2 = (gcnew System::Windows::Forms::ComboBox());
				 this->Btn_Tbox_Connect = (gcnew System::Windows::Forms::Button());
				 this->serialPort_Tbox = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->label1 = (gcnew System::Windows::Forms::Label());
				 this->Txt_CarSpeed = (gcnew System::Windows::Forms::Label());
				 this->label2 = (gcnew System::Windows::Forms::Label());
				 this->label3 = (gcnew System::Windows::Forms::Label());
				 this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
				 this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
				 this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
				 this->label5 = (gcnew System::Windows::Forms::Label());
				 this->Btn_Radar_Connect = (gcnew System::Windows::Forms::Button());
				 this->comboBox3 = (gcnew System::Windows::Forms::ComboBox());
				 this->label4 = (gcnew System::Windows::Forms::Label());
				 this->serialPort_Radar = (gcnew System::IO::Ports::SerialPort(this->components));
				 this->Btn_Refresh_Combox = (gcnew System::Windows::Forms::Button());
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->BeginInit();
				 this->groupBox1->SuspendLayout();
				 this->groupBox2->SuspendLayout();
				 this->groupBox3->SuspendLayout();
				 this->SuspendLayout();
				 // 
				 // chart1
				 // 
				 chartArea1->AxisX->Maximum = 500;
				 chartArea1->AxisX->Minimum = -500;
				 chartArea1->AxisY->Maximum = 500;
				 chartArea1->AxisY->Minimum = 0;
				 chartArea1->Name = L"ChartArea1";
				 this->chart1->ChartAreas->Add(chartArea1);
				 legend1->Name = L"Legend1";
				 this->chart1->Legends->Add(legend1);
				 this->chart1->Location = System::Drawing::Point(240, 12);
				 this->chart1->Name = L"chart1";
				 series1->ChartArea = L"ChartArea1";
				 series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series1->Legend = L"Legend1";
				 series1->Name = L"LiDAR";
				 series2->ChartArea = L"ChartArea1";
				 series2->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Point;
				 series2->Legend = L"Legend1";
				 series2->MarkerColor = System::Drawing::Color::Red;
				 series2->Name = L"Radar";
				 this->chart1->Series->Add(series1);
				 this->chart1->Series->Add(series2);
				 this->chart1->Size = System::Drawing::Size(943, 634);
				 this->chart1->TabIndex = 0;
				 this->chart1->Text = L"chart1";
				 // 
				 // comboBox1
				 // 
				 this->comboBox1->FormattingEnabled = true;
				 this->comboBox1->Location = System::Drawing::Point(6, 50);
				 this->comboBox1->Name = L"comboBox1";
				 this->comboBox1->Size = System::Drawing::Size(100, 20);
				 this->comboBox1->TabIndex = 1;
				 // 
				 // Btn_LiDAR_Connected
				 // 
				 this->Btn_LiDAR_Connected->Location = System::Drawing::Point(121, 28);
				 this->Btn_LiDAR_Connected->Name = L"Btn_LiDAR_Connected";
				 this->Btn_LiDAR_Connected->Size = System::Drawing::Size(75, 23);
				 this->Btn_LiDAR_Connected->TabIndex = 2;
				 this->Btn_LiDAR_Connected->Text = L"連接";
				 this->Btn_LiDAR_Connected->UseVisualStyleBackColor = true;
				 this->Btn_LiDAR_Connected->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDAR_Connected_Click);
				 // 
				 // serialPort_LiDAR
				 // 
				 this->serialPort_LiDAR->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_LiDAR_DataReceived);
				 // 
				 // timer1
				 // 
				 this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
				 // 
				 // Btn_LiDAR_DisConnect
				 // 
				 this->Btn_LiDAR_DisConnect->Location = System::Drawing::Point(121, 68);
				 this->Btn_LiDAR_DisConnect->Name = L"Btn_LiDAR_DisConnect";
				 this->Btn_LiDAR_DisConnect->Size = System::Drawing::Size(75, 23);
				 this->Btn_LiDAR_DisConnect->TabIndex = 3;
				 this->Btn_LiDAR_DisConnect->Text = L"關閉";
				 this->Btn_LiDAR_DisConnect->UseVisualStyleBackColor = true;
				 this->Btn_LiDAR_DisConnect->Click += gcnew System::EventHandler(this, &MyForm::Btn_LiDAR_DisConnect_Click);
				 // 
				 // comboBox2
				 // 
				 this->comboBox2->FormattingEnabled = true;
				 this->comboBox2->Location = System::Drawing::Point(6, 24);
				 this->comboBox2->Name = L"comboBox2";
				 this->comboBox2->Size = System::Drawing::Size(100, 20);
				 this->comboBox2->TabIndex = 1;
				 // 
				 // Btn_Tbox_Connect
				 // 
				 this->Btn_Tbox_Connect->Location = System::Drawing::Point(121, 21);
				 this->Btn_Tbox_Connect->Name = L"Btn_Tbox_Connect";
				 this->Btn_Tbox_Connect->Size = System::Drawing::Size(75, 23);
				 this->Btn_Tbox_Connect->TabIndex = 4;
				 this->Btn_Tbox_Connect->Text = L"連接";
				 this->Btn_Tbox_Connect->UseVisualStyleBackColor = true;
				 this->Btn_Tbox_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_Tbox_Connect_Click);
				 // 
				 // serialPort_Tbox
				 // 
				 this->serialPort_Tbox->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Tbox_DataReceived);
				 // 
				 // label1
				 // 
				 this->label1->AutoSize = true;
				 this->label1->Location = System::Drawing::Point(6, 62);
				 this->label1->Name = L"label1";
				 this->label1->Size = System::Drawing::Size(56, 12);
				 this->label1->TabIndex = 5;
				 this->label1->Text = L"目前車速:";
				 // 
				 // Txt_CarSpeed
				 // 
				 this->Txt_CarSpeed->AutoSize = true;
				 this->Txt_CarSpeed->Location = System::Drawing::Point(82, 62);
				 this->Txt_CarSpeed->Name = L"Txt_CarSpeed";
				 this->Txt_CarSpeed->Size = System::Drawing::Size(11, 12);
				 this->Txt_CarSpeed->TabIndex = 6;
				 this->Txt_CarSpeed->Text = L"0";
				 // 
				 // label2
				 // 
				 this->label2->AutoSize = true;
				 this->label2->Location = System::Drawing::Point(16, 79);
				 this->label2->Name = L"label2";
				 this->label2->Size = System::Drawing::Size(33, 12);
				 this->label2->TabIndex = 7;
				 this->label2->Text = L"label2";
				 // 
				 // label3
				 // 
				 this->label3->AutoSize = true;
				 this->label3->Location = System::Drawing::Point(16, 85);
				 this->label3->Name = L"label3";
				 this->label3->Size = System::Drawing::Size(33, 12);
				 this->label3->TabIndex = 8;
				 this->label3->Text = L"label3";
				 // 
				 // groupBox1
				 // 
				 this->groupBox1->Controls->Add(this->comboBox1);
				 this->groupBox1->Controls->Add(this->Btn_LiDAR_Connected);
				 this->groupBox1->Controls->Add(this->label2);
				 this->groupBox1->Controls->Add(this->Btn_LiDAR_DisConnect);
				 this->groupBox1->Location = System::Drawing::Point(12, 89);
				 this->groupBox1->Name = L"groupBox1";
				 this->groupBox1->Size = System::Drawing::Size(200, 100);
				 this->groupBox1->TabIndex = 9;
				 this->groupBox1->TabStop = false;
				 this->groupBox1->Text = L"光達";
				 // 
				 // groupBox2
				 // 
				 this->groupBox2->Controls->Add(this->comboBox2);
				 this->groupBox2->Controls->Add(this->Btn_Tbox_Connect);
				 this->groupBox2->Controls->Add(this->Txt_CarSpeed);
				 this->groupBox2->Controls->Add(this->label3);
				 this->groupBox2->Controls->Add(this->label1);
				 this->groupBox2->Location = System::Drawing::Point(12, 223);
				 this->groupBox2->Name = L"groupBox2";
				 this->groupBox2->Size = System::Drawing::Size(200, 100);
				 this->groupBox2->TabIndex = 10;
				 this->groupBox2->TabStop = false;
				 this->groupBox2->Text = L"Tbox";
				 // 
				 // groupBox3
				 // 
				 this->groupBox3->Controls->Add(this->label5);
				 this->groupBox3->Controls->Add(this->Btn_Radar_Connect);
				 this->groupBox3->Controls->Add(this->comboBox3);
				 this->groupBox3->Controls->Add(this->label4);
				 this->groupBox3->Location = System::Drawing::Point(12, 360);
				 this->groupBox3->Name = L"groupBox3";
				 this->groupBox3->Size = System::Drawing::Size(200, 84);
				 this->groupBox3->TabIndex = 11;
				 this->groupBox3->TabStop = false;
				 this->groupBox3->Text = L"Radar";
				 // 
				 // label5
				 // 
				 this->label5->AutoSize = true;
				 this->label5->Location = System::Drawing::Point(121, 66);
				 this->label5->Name = L"label5";
				 this->label5->Size = System::Drawing::Size(33, 12);
				 this->label5->TabIndex = 14;
				 this->label5->Text = L"label5";
				 // 
				 // Btn_Radar_Connect
				 // 
				 this->Btn_Radar_Connect->Location = System::Drawing::Point(121, 21);
				 this->Btn_Radar_Connect->Name = L"Btn_Radar_Connect";
				 this->Btn_Radar_Connect->Size = System::Drawing::Size(75, 23);
				 this->Btn_Radar_Connect->TabIndex = 13;
				 this->Btn_Radar_Connect->Text = L"連接";
				 this->Btn_Radar_Connect->UseVisualStyleBackColor = true;
				 this->Btn_Radar_Connect->Click += gcnew System::EventHandler(this, &MyForm::Btn_Radar_Connect_Click);
				 // 
				 // comboBox3
				 // 
				 this->comboBox3->FormattingEnabled = true;
				 this->comboBox3->Location = System::Drawing::Point(6, 21);
				 this->comboBox3->Name = L"comboBox3";
				 this->comboBox3->Size = System::Drawing::Size(100, 20);
				 this->comboBox3->TabIndex = 12;
				 // 
				 // label4
				 // 
				 this->label4->AutoSize = true;
				 this->label4->Location = System::Drawing::Point(16, 69);
				 this->label4->Name = L"label4";
				 this->label4->Size = System::Drawing::Size(33, 12);
				 this->label4->TabIndex = 7;
				 this->label4->Text = L"label4";
				 // 
				 // serialPort_Radar
				 // 
				 this->serialPort_Radar->DataReceived += gcnew System::IO::Ports::SerialDataReceivedEventHandler(this, &MyForm::serialPort_Radar_DataReceived);
				 // 
				 // Btn_Refresh_Combox
				 // 
				 this->Btn_Refresh_Combox->Location = System::Drawing::Point(133, 50);
				 this->Btn_Refresh_Combox->Name = L"Btn_Refresh_Combox";
				 this->Btn_Refresh_Combox->Size = System::Drawing::Size(75, 23);
				 this->Btn_Refresh_Combox->TabIndex = 12;
				 this->Btn_Refresh_Combox->Text = L"更新列表";
				 this->Btn_Refresh_Combox->UseVisualStyleBackColor = true;
				 this->Btn_Refresh_Combox->Click += gcnew System::EventHandler(this, &MyForm::Btn_Refresh_Combox_Click);
				 // 
				 // MyForm
				 // 
				 this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
				 this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				 this->ClientSize = System::Drawing::Size(1223, 667);
				 this->Controls->Add(this->Btn_Refresh_Combox);
				 this->Controls->Add(this->groupBox3);
				 this->Controls->Add(this->groupBox2);
				 this->Controls->Add(this->groupBox1);
				 this->Controls->Add(this->chart1);
				 this->Name = L"MyForm";
				 this->Text = L"MyForm";
				 (cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->chart1))->EndInit();
				 this->groupBox1->ResumeLayout(false);
				 this->groupBox1->PerformLayout();
				 this->groupBox2->ResumeLayout(false);
				 this->groupBox2->PerformLayout();
				 this->groupBox3->ResumeLayout(false);
				 this->groupBox3->PerformLayout();
				 this->ResumeLayout(false);

			 }
#pragma endregion

#pragma region 視窗更新
	private:void ComPortRefresh(void)
	{
		comboBox1->Items->Clear();
		comboBox2->Items->Clear();
		comboBox3->Items->Clear();
		cli::array<System::String^>^ Port = SerialPort::GetPortNames();
		comboBox1->Items->AddRange(Port);
		comboBox2->Items->AddRange(Port);
		comboBox3->Items->AddRange(Port);
	}
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
		chart1->Series["LiDAR"]->Points->Clear();
		chart1->Series["Radar"]->Points->Clear();
		if (getLiDARData)
		{
			for (uint i = 0; i < 361; i++)
				chart1->Series["LiDAR"]->Points->AddXY(LIDAR_cooridate[i].x, LIDAR_cooridate[i].y);
			vector<int>Lab;
			int a = partition(LIDAR_cooridate, Lab);
		}
		Txt_CarSpeed->Text = Current_Speed.ToString();
		chart1->Series["Radar"]->Points->AddXY(R_Data.X, R_Data.Y);
		if (serialPort_LiDAR->IsOpen)
		{
			label2->ForeColor = Color::Green;
			label2->Text = "Connected";
		}
		else
		{
			label2->ForeColor = Color::Red;
			label2->Text = "fail";
		}

		if (serialPort_Tbox->IsOpen)
		{
			label3->ForeColor = Color::Green;
			label3->Text = "Connected";
		}
		else {
			label3->ForeColor = Color::Red;
			label3->Text = "fail";
		}

		if (serialPort_Radar->IsOpen)
		{
			label4->ForeColor = Color::Green;
			label4->Text = "Connected";
		}
		else {
			label4->ForeColor = Color::Red;
			label4->Text = "fail";
		}
		switch (R_Data.Mod)
		{
		case 0x01:
			label5->Text = "BSD Mode";
			label5->Refresh();
			break;
		case 0x02:
			label5->Text = "RCTA Mode";
			label5->Refresh();
			break;
		case 0x03:
			label5->Text = "DOW Mode";
			label5->Refresh();
			break;
		}

	}
#pragma endregion

#pragma region 按鈕(觸發事件)
	private: System::Void Btn_LiDAR_DisConnect_Click(System::Object^  sender, System::EventArgs^  e) {
		timer1->Stop();
		cli::array<System::Byte>^ LMS_Stope_manage = gcnew cli::array<System::Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x25, 0x38, 0x08};
		serialPort_LiDAR->Write(LMS_Stope_manage, 0, 8);
		serialPort_LiDAR->Close();
	}
	private: System::Void Btn_Tbox_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		if (serialPort_Tbox->IsOpen)
		{
			serialPort_Tbox->Close();
			Sleep(10);
		}

		serialPort_Tbox->PortName = comboBox2->Text;
		serialPort_Tbox->BaudRate = 115200;
		serialPort_Tbox->DataBits = 8;
		serialPort_Tbox->StopBits = StopBits::One;
		serialPort_Tbox->Parity = Parity::None;
		serialPort_Tbox->Open();
	}
	private: System::Void Btn_Refresh_Combox_Click(System::Object^  sender, System::EventArgs^  e) {
		ComPortRefresh();
	}
	private: System::Void Btn_Radar_Connect_Click(System::Object^  sender, System::EventArgs^  e) {
		serialPort_Radar->PortName = comboBox3->Text;
		serialPort_Radar->BaudRate = 460800;
		serialPort_Radar->DataBits = 8;
		serialPort_Radar->StopBits = StopBits::One;
		serialPort_Radar->Parity = Parity::None;
		serialPort_Radar->Open();
	}
	private: System::Void Btn_LiDAR_Connected_Click(System::Object^  sender, System::EventArgs^  e) {

		serialPort_LiDAR->PortName = comboBox1->Text;
		serialPort_LiDAR->Encoding = System::Text::Encoding::GetEncoding(28591);
		serialPort_LiDAR->BaudRate = 9600;
		serialPort_LiDAR->DataBits = 8;
		serialPort_LiDAR->StopBits = StopBits::One;
		serialPort_LiDAR->Parity = Parity::None;
		serialPort_LiDAR->Open();

		cli::array<System::Byte>^ LMS_Angular_range_change_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x05, 0x00, 0x3B, 0xB4, 0x00, 0x32, 0x00, 0x3B, 0x1F };//更改LMS經度0.5度
		cli::array<System::Byte>^ continuous_LMS_data_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x24, 0x34, 0x08 };//更改成連續指令緩區
		cli::array<System::Byte>^ LMS_baundrate_500k_manage = gcnew cli::array<Byte>{ 0x02, 0x00, 0x02, 0x00, 0x20, 0x48, 0x58, 0x08 };//更改包率

		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_baundrate_500k_manage, 0, 8);
			Sleep(500);
			serialPort_LiDAR->Close();
			serialPort_LiDAR->BaudRate = 500000;
			serialPort_LiDAR->Open();
		}
		if (serialPort_LiDAR->IsOpen)
		{
			serialPort_LiDAR->Write(LMS_Angular_range_change_manage, 0, 11);
			_sleep(500);
			serialPort_LiDAR->Write(continuous_LMS_data_manage, 0, 8);
		}
		timeBeginPeriod(1);
		mHandler = gcnew TimerEventHandler(this, &MyForm::TimerCallback);
		mTimerId = timeSetEvent(1, 0, mHandler, IntPtr::Zero, EVENT_TYPE);
		mTestStart = DateTime::Now;
		mTestTick = 0;
	}
#pragma endregion

#pragma region SerialPort(DataReceived)
	private: System::Void serialPort_Tbox_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		cli::array<System::Byte>^bTboxData = gcnew cli::array<Byte>(format);

		serialPort_Tbox->Read(bTboxData, 0, 1);
		if (bTboxData[0] == 0x80)
		{
			byte Checksum = 0;
			serialPort_Tbox->Read(bTboxData, 1, format - 1);
			for (uint i = 0; i < format - 1; i++)
			{
				Checksum += bTboxData[i];
			}
			if (Checksum == bTboxData[format - 1])
			{
				Current_Speed = bTboxData[9];
			}

		}
		if (serialPort_Tbox->BytesToRead >= format * 2)
		{
			serialPort_Tbox->DiscardInBuffer();

		}
	}
	private: System::Void serialPort_LiDAR_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {

		cli::array<System::Byte>^ LiDAR_SerialPortData = gcnew cli::array<Byte>(10000);
		int ReadSize = serialPort_LiDAR->Read(LiDAR_SerialPortData, 0, 10000);

		for (int i = 0; i < ReadSize; i++)
		{
			if ((LiDAR_SerialPortData[i] == 0x02) && (LiDAR_SerialPortData[i + 1] == 0x80) && (LiDAR_SerialPortData[i + 4] == 0xB0) && (GetHeader == false))
			{

				for (int j = i; j < ReadSize; j++)
				{
					LiDAR_Data[counter] = LiDAR_SerialPortData[j];
					counter++;
					if ((counter + 1) == 734)
						break;
				}
				GetHeader = true;
				break;
			}
			if (GetHeader)
			{
				if ((counter + 1) == 734)
					break;
				LiDAR_Data[counter] = LiDAR_SerialPortData[i];
				counter++;
			}
		}
		int Data[361];


		if ((counter + 1) == 734)
		{
			for (uint i = 0; i < 361; i++)
			{
				Data[i] = LiDAR_Data[2 * i + 7] + (LiDAR_Data[8 + i * 2] & 0x1F) * 256;
			}
			for (uint i = 0; i < 361; i++)
			{

				Pt p;
				p.x = (Data[i]) * cos((0.5 * i) * (M_PI / 180));
				p.y = (Data[i]) * sin((0.5 * i) * (M_PI / 180));
				LIDAR_cooridate.push_back(p);
			}

			GetHeader = false;
			counter = 0;
		}
	}
	private: System::Void serialPort_Radar_DataReceived(System::Object^  sender, System::IO::Ports::SerialDataReceivedEventArgs^  e) {
		uint Radar_buff_size = 12;
		byte bChecksum = 0;
		if (serialPort_Radar->BytesToRead < Radar_buff_size)
			return;
		cli::array<System::Byte>^ Radar_buff = gcnew cli::array<Byte>(Radar_buff_size);
		serialPort_Radar->Read(Radar_buff, 0, Radar_buff_size);

		if (Radar_buff[0] == 0x54)
		{
			for (uint i = 0; i < Radar_buff_size - 1; i++)
				bChecksum += Radar_buff[i];
			if (bChecksum == Radar_buff[11])
			{
				R_Data.ID = Radar_buff[7];
				double theta = Radar_buff[10] * M_PI / 180;
				R_Data.X = 100 * Radar_buff[8] * Math::Cos(theta);
				R_Data.Y = 100 * Radar_buff[8] * Math::Sin(theta);
				R_Data.Velocity = Radar_buff[9];
				R_Data.theta = Radar_buff[10];
				R_Data.Mod = Radar_buff[3];
			}
		}

	}
#pragma endregion
	private:bool predicate(Pt P1, Pt P2)
	{
		return ((P1.x - P2.x)*(P1.x - P2.x) + (P1.y - P2.y)*(P1.y - P2.y)) <= 5;
	}
	private:int partition(vector<Pt>& _vec, vector<int>& labels)
	{
		int i, j, N = _vec.size();
		const Pt* vec = &_vec[0];

		const int PARENT = 0;
		const int RANK = 1;

		vector<int> _nodes(N * 2);
		int(*nodes)[2] = (int(*)[2])&_nodes[0];

		for (i = 0; i < N; i++)
		{
			nodes[i][PARENT] = -1;
			nodes[i][RANK] = 0;
		}
		for (i = 0; i < N; i++)
		{
			int root = i;

			// find root
			while (nodes[root][PARENT] >= 0)
				root = nodes[root][PARENT];

			for (j = 0; j < N; j++)
			{
				if (i == j || !predicate(vec[i], vec[j]))
					continue;
				int root2 = j;

				while (nodes[root2][PARENT] >= 0)
					root2 = nodes[root2][PARENT];

				if (root2 != root)
				{
					// unite both trees
					int rank = nodes[root][RANK], rank2 = nodes[root2][RANK];
					if (rank > rank2)
						nodes[root2][PARENT] = root;
					else
					{
						nodes[root][PARENT] = root2;
						nodes[root2][RANK] += rank == rank2;
						root = root2;
					}
					//assert(nodes[root][PARENT] < 0);

					int k = j, parent;

					// compress the path from node2 to root
					while ((parent = nodes[k][PARENT]) >= 0)
					{
						nodes[k][PARENT] = root;
						k = parent;
					}

					// compress the path from node to root
					k = i;
					while ((parent = nodes[k][PARENT]) >= 0)
					{
						nodes[k][PARENT] = root;
						k = parent;
					}
				}
			}
		}
		for (unsigned int i = 0; i < N; i++)
			labels.push_back(0);
		int nclasses = 0;

		for (i = 0; i < N; i++)
		{
			int root = i;
			while (nodes[root][PARENT] >= 0)
				root = nodes[root][PARENT];
			if (nodes[root][RANK] >= 0)
				nodes[root][RANK] = ~nclasses++;
			labels[i] = ~nodes[root][RANK];
		}
		return nclasses;
	}
	private:void TimerCallback(int id, int msg, IntPtr user, int dw1, int dw2)
	{
		mTestTick += 1;

		if (((mTestTick % 200) == 0) && (mTimerId != 0))

			this->BeginInvoke(gcnew TestEventHandler(this, &MyForm::TimerTick), mTestTick, DateTime::Now - mTestStart);
	}
	private:void TimerTick(int msec, TimeSpan span)
	{
	
	}
	};
}
