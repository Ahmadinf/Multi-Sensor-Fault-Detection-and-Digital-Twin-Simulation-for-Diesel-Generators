using System;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;
using System.Windows.Forms.DataVisualization.Charting;
using ChartingSeries = System.Windows.Forms.DataVisualization.Charting.Series;
using MathNet.Numerics.IntegralTransforms;
using System.Collections.Generic;

namespace Simulatordiesel
{
    // Helper class to define and manage sensor parameters
    public class SensorParameterConfig
    {
        public string Name { get; set; }
        public double CurrentValue { get; set; }
        public int TrackBarMin { get; set; }
        public int TrackBarMax { get; set; }
        public int TrackBarDefault { get; set; }
        public double ScalingFactor { get; set; }
        public string ValueFormat { get; set; }

        public Label ParameterLabel { get; set; }
        public TrackBar ParameterTrackBar { get; set; }

        public SensorParameterConfig(string name, int tbMin, int tbMax, int tbDefault, double scalingFactor, string valueFormat)
        {
            Name = name;
            TrackBarMin = tbMin;
            TrackBarMax = tbMax;
            TrackBarDefault = tbDefault;
            ScalingFactor = scalingFactor;
            CurrentValue = tbDefault * scalingFactor;
            ValueFormat = valueFormat;
        }

        public void UpdateValueFromTrackBar()
        {
            CurrentValue = ParameterTrackBar.Value * ScalingFactor;
            ParameterLabel.Text = $"{Name}: {CurrentValue.ToString(ValueFormat)}";
        }
    }

    public partial class Form1 : Form
    {
        private Random random = new Random();
        private const double DefaultSampleRate = 1000.0; // Default sample rate for most sensors
        private const double SimulationDuration = 1.0;

        // --- Konstanta Global untuk Sensor Akustik ---
        private const double S_ACOUSTIC = 0.0126; // V/Pa
        private const double RS_ACOUSTIC = 300.0; // Ohms
        private const double RL_ACOUSTIC_CONSTANT = 1000000.0; // 1 MΩ
        private const double ENGINE_FREQ = 25.0; // Hz
        private const double COMBUSTION_FREQ = 50.0; // Hz
        private const double VIBRATION_FREQ = 120.0; // Hz
        private const double ACOUSTIC_SAMPLING_RATE = 40000.0; // 40 kHz
        // ---------------------------------------------

        // --- Konstanta Global untuk Sensor Akselerometer (ADXL335) ---
        private const double V0_ADXL335 = 1.5; // V, Zero-g Offset
        private const double S_ADXL335 = 0.300; // V/g
        private const double ACCELEROMETER_GETARAN_FREQ = 50.0; // Hz
        private const double GRAVITY_CONSTANT_VALUE = 1.0; // 1g
        private const double ACCELEROMETER_SAMPLING_RATE = 3200.0; // 3.2 kHz
        // ---------------------------------------------

        // --- Konstanta Global untuk Sensor Suhu (DS18B20) ---
        private const double LSB_DS18B20 = 0.0625; // 1/16 °C
        private const int RAW_MIN_LIMIT = -880;
        private const int RAW_MAX_LIMIT = 2000;
        // ----------------------------------------------------

        // --- Konstanta Global untuk Sensor Kualitas Oli ---
        private const double ER_OIL_QUALITY_CONSTANT = 2.3; // Permitivitas relatif
        private const double A_OIL_QUALITY_CONSTANT = 2e-4; // Luas (m²)
        private const double D_OIL_QUALITY_CONSTANT = 2e-3; // Jarak (m)
        // --------------------------------------------------------------------------------

        // --- Konstanta Global untuk Sensor RPM ---
        private const double RPM_CONVERSION_FACTOR = 60.0;
        // -----------------------------------------

        // --- Konstanta Global untuk Sistem Dinamis (Poles & Zeros) ---
        private const double SYSTEM_SAMPLING_PERIOD_TS = 0.1; // 10 Hz sampling for discrete model
        // -----------------------------------------------------------


        // --- Deklarasi Kontrol UI Umum ---
        private TableLayoutPanel tableLayoutPanelMain;
        private TableLayoutPanel scrollableSensorContainer;

        // GroupBoxes
        private GroupBox groupBoxAcoustic;
        private GroupBox groupBoxAccelerometer;
        private GroupBox groupBoxTemperature;
        private GroupBox groupBoxOilQuality;
        private GroupBox groupBoxRpm;
        private GroupBox groupBoxSystemPlots;

        // Labels Rumus
        private Label lblAcousticFormula;
        private Label lblAccelerometerFormula;
        private Label lblTemperatureFormula;
        private Label lblOilQualityFormula;
        private Label lblRpmFormula;
        private Label lblOilQualityNotationExplanation;
        private Label lblSystemOverallFormula;

        // Charts
        private Chart chartAcousticTime;
        private Chart chartAcousticFreq;
        private Chart chartAccelerometerTime;
        private Chart chartAccelerometerFreq;
        private Chart chartTemperatureTime;
        private Chart chartTemperatureFreq;
        private Chart chartOilQualityTime;
        private Chart chartOilQualityFreq;
        private Chart chartRpmTime;
        private Chart chartRpmFreq;

        // Charts Sistem
        private Chart chartSDomain;
        private Chart chartZDomain;
        private PictureBox pbSystemImage;

        // List parameter
        private List<SensorParameterConfig> _acousticParams;
        private List<SensorParameterConfig> _accelerometerParams;
        private List<SensorParameterConfig> _temperatureParams;
        private List<SensorParameterConfig> _oilQualityParams;
        private List<SensorParameterConfig> _rpmParams;

        // List dinamis untuk menyimpan poles dan zeros sistem
        private List<Complex> _dynamicSPoles;
        private List<Complex> _dynamicZPoles;
        private List<Complex> _dynamicZZeros;


        public Form1()
        {
            this.Text = "Diesel Generator Simulator";
            this.Size = new Size(1800, 1000);
            this.StartPosition = FormStartPosition.CenterScreen;
            this.AutoScroll = false;

            _dynamicSPoles = new List<Complex>();
            _dynamicZPoles = new List<Complex>();
            _dynamicZZeros = new List<Complex>();

            SetupUIAndChartsProgrammatically();

            // --- Memuat gambar 3D ke PictureBox ---
            // Pastikan path file gambar ini benar di komputer Anda
            string image3dPath = @"D:\Kuliah\Semester 3\Sistem Pengolahan Sinyal\3D\Fix1.png";
            try
            {
                if (System.IO.File.Exists(image3dPath))
                {
                    pbSystemImage.Image = Image.FromFile(image3dPath);
                }
            }
            catch (Exception) { }
            // ------------------------------------

            // Inisialisasi awal simulasi
            UpdateAcousticSensor();
            UpdateAccelerometerSensor();
            UpdateTemperatureSensor();
            UpdateOilQualitySensor();
            UpdateRpmSensor();
            UpdateSystemPlots();
            UpdateSystemOverallFormula();
        }

        private void SetupUIAndChartsProgrammatically()
        {
            // --- 1. TableLayoutPanel Utama ---
            tableLayoutPanelMain = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(5)
            };
            tableLayoutPanelMain.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 70F));
            tableLayoutPanelMain.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30F));
            this.Controls.Add(tableLayoutPanelMain);

            // --- 1.1. Container Scrollable untuk Sensor ---
            scrollableSensorContainer = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                AutoScroll = true,
                Padding = new Padding(0, 0, 20, 0)
            };
            tableLayoutPanelMain.Controls.Add(scrollableSensorContainer, 0, 0);

            // --- 2. Inisialisasi Parameter ---
            _acousticParams = new List<SensorParameterConfig>
            {
                new SensorParameterConfig("P_engine", 0, 100, 10, 0.01, "F2"),
                new SensorParameterConfig("P_combustion", 0, 100, 5, 0.005, "F3"),
                new SensorParameterConfig("P_vibration", 0, 100, 7, 0.007, "F3")
            };

            _accelerometerParams = new List<SensorParameterConfig>
            {
                new SensorParameterConfig("a_getaran", -300, 300, 0, 0.01, "F2")
            };

            _temperatureParams = new List<SensorParameterConfig>
            {
                new SensorParameterConfig("Raw", RAW_MIN_LIMIT, RAW_MAX_LIMIT, 400, 1.0, "F0")
            };

            _oilQualityParams = new List<SensorParameterConfig>
            {
                new SensorParameterConfig("Permittivity ε0", 10, 150, 88, 1e-13, "E2")
            };

            _rpmParams = new List<SensorParameterConfig>
            {
                new SensorParameterConfig("Count", 0, 200, 100, 1.0, "F0"),
                new SensorParameterConfig("T_win (s)", 1, 100, 10, 0.1, "F1"),
                new SensorParameterConfig("N (Pulses/Rev)", 1, 10, 4, 1.0, "F0")
            };

            // --- 3. Setup Sensor GroupBoxes ---
            SetupSensorGroupBox(scrollableSensorContainer, out groupBoxAcoustic, out lblAcousticFormula, _acousticParams, out chartAcousticTime, out chartAcousticFreq, "Acoustic (AMM-3738-B-R)", tbAcoustic_Scroll, "Maximum Frequency: 20-20,000 Hz");
            SetupSensorGroupBox(scrollableSensorContainer, out groupBoxAccelerometer, out lblAccelerometerFormula, _accelerometerParams, out chartAccelerometerTime, out chartAccelerometerFreq, "Accelerometer (ADXL335)", tbAccelerometer_Scroll, "Maximum Frequency (X/Y): 0.5-1,600 Hz, (Z): 0.5-550 Hz");
            SetupSensorGroupBox(scrollableSensorContainer, out groupBoxTemperature, out lblTemperatureFormula, _temperatureParams, out chartTemperatureTime, out chartTemperatureFreq, "Temperature (DS18B20)", tbTemperature_Scroll);
            SetupSensorGroupBox(scrollableSensorContainer, out groupBoxOilQuality, out lblOilQualityFormula, _oilQualityParams, out chartOilQualityTime, out chartOilQualityFreq, "Oil Quality (Capacitive probe + AD7745)", tbOilQuality_Scroll);
            SetupSensorGroupBox(scrollableSensorContainer, out groupBoxRpm, out lblRpmFormula, _rpmParams, out chartRpmTime, out chartRpmFreq, "RPM (Hall Effect-A3144)", tbRpm_Scroll);

            // Label Penjelasan Oil
            TableLayoutPanel oilQualityLeftControlPanel = (TableLayoutPanel)groupBoxOilQuality.Controls[0].Controls[0];
            lblOilQualityNotationExplanation = new Label
            {
                Text = "",
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.TopLeft,
                Padding = new Padding(0, 5, 0, 0),
                AutoSize = true,
                Font = new Font(this.Font.FontFamily, 8),
                ForeColor = Color.DimGray
            };
            oilQualityLeftControlPanel.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            oilQualityLeftControlPanel.Controls.Add(lblOilQualityNotationExplanation, 0, oilQualityLeftControlPanel.RowCount - 1);
            oilQualityLeftControlPanel.SetColumnSpan(lblOilQualityNotationExplanation, 2);

            // --- 4. Setup System Plots GroupBox ---
            groupBoxSystemPlots = new GroupBox
            {
                Text = "System Overview & Analysis",
                Dock = DockStyle.Fill,
                Padding = new Padding(5)
            };
            tableLayoutPanelMain.Controls.Add(groupBoxSystemPlots, 1, 0);

            TableLayoutPanel systemOverviewPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 4,
                Padding = new Padding(5)
            };
            systemOverviewPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 20F));
            systemOverviewPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 25F));
            systemOverviewPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 27.5F));
            systemOverviewPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 27.5F));
            groupBoxSystemPlots.Controls.Add(systemOverviewPanel);

            // PictureBox
            pbSystemImage = new PictureBox
            {
                Dock = DockStyle.Fill,
                SizeMode = PictureBoxSizeMode.Zoom,
                BorderStyle = BorderStyle.FixedSingle,
            };
            systemOverviewPanel.Controls.Add(pbSystemImage, 0, 0);

            // Label Rumus Sistem
            lblSystemOverallFormula = new Label
            {
                Text = "Memuat Rumus Sistem...",
                AutoSize = false,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.TopLeft,
                Padding = new Padding(5),
                Font = new Font(this.Font.FontFamily, 9, FontStyle.Regular),
                BorderStyle = BorderStyle.FixedSingle,
                AutoEllipsis = false
            };
            systemOverviewPanel.Controls.Add(lblSystemOverallFormula, 0, 1);

            // S-Domain Chart
            chartSDomain = new Chart { Dock = DockStyle.Fill };
            ConfigureChart(chartSDomain, "S-Domain (Stabilitas)", "Real Axis", "Imaginary Axis", false);
            systemOverviewPanel.Controls.Add(chartSDomain, 0, 2);

            // Z-Domain Chart
            chartZDomain = new Chart { Dock = DockStyle.Fill };
            ConfigureChart(chartZDomain, "Z-Domain (Poles & Zeros)", "Real Axis", "Imaginary Axis", false);
            systemOverviewPanel.Controls.Add(chartZDomain, 0, 3);

            ConfigureSystemChartScales();
        }

        private void ConfigureSystemChartScales()
        {
            // --- Konfigurasi S-Domain ---
            var areaS = chartSDomain.ChartAreas[0];

            // Setup Range (Skala)
            areaS.AxisX.Minimum = -3; areaS.AxisX.Maximum = 3;
            areaS.AxisY.Minimum = -3; areaS.AxisY.Maximum = 3;
            areaS.AxisX.Interval = 1; areaS.AxisY.Interval = 1;

            // --- GARIS KARTESIUS (CROSS DI 0,0) S-DOMAIN ---
            // Sumbu X memotong Y di 0, dan sebaliknya
            areaS.AxisX.Crossing = 0;
            areaS.AxisY.Crossing = 0;

            // Garis sumbu utama hitam tebal
            areaS.AxisX.LineWidth = 2;
            areaS.AxisY.LineWidth = 2;
            areaS.AxisX.LineColor = Color.Black;
            areaS.AxisY.LineColor = Color.Black;

            // Grid latar belakang lebih tipis (abu-abu) agar sumbu utama terlihat jelas
            areaS.AxisX.MajorGrid.LineColor = Color.LightGray;
            areaS.AxisY.MajorGrid.LineColor = Color.LightGray;


            // --- Konfigurasi Z-Domain ---
            var areaZ = chartZDomain.ChartAreas[0];

            // Setup Range (Skala)
            areaZ.AxisX.Minimum = -1.5; areaZ.AxisX.Maximum = 1.5;
            areaZ.AxisY.Minimum = -1.5; areaZ.AxisY.Maximum = 1.5;
            areaZ.AxisX.Interval = 0.5; areaZ.AxisY.Interval = 0.5;

            // --- GARIS KARTESIUS (CROSS DI 0,0) Z-DOMAIN ---
            areaZ.AxisX.Crossing = 0;
            areaZ.AxisY.Crossing = 0;

            // Garis sumbu utama hitam tebal
            areaZ.AxisX.LineWidth = 2;
            areaZ.AxisY.LineWidth = 2;
            areaZ.AxisX.LineColor = Color.Black;
            areaZ.AxisY.LineColor = Color.Black;

            // Grid latar belakang lebih tipis
            areaZ.AxisX.MajorGrid.LineColor = Color.LightGray;
            areaZ.AxisY.MajorGrid.LineColor = Color.LightGray;
        }

        private void SetupSensorGroupBox(TableLayoutPanel parentPanel, out GroupBox groupBox, out Label formulaLabel,
                                         List<SensorParameterConfig> paramConfigs,
                                         out Chart chartTime, out Chart chartFreq,
                                         string groupText, EventHandler scrollHandler,
                                         string maxFreqInfo = null)
        {
            groupBox = new GroupBox
            {
                Text = groupText,
                Dock = DockStyle.Top,
                Padding = new Padding(5)
            };
            parentPanel.Controls.Add(groupBox);

            TableLayoutPanel splitPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(0)
            };
            splitPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 280F));
            splitPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            groupBox.Controls.Add(splitPanel);

            TableLayoutPanel leftControlPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1 + paramConfigs.Count + 1,
                Padding = new Padding(0)
            };
            splitPanel.Controls.Add(leftControlPanel, 0, 0);

            leftControlPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 100F));
            leftControlPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));

            const int formulaHeight = 80;
            const int paramHeight = 30;

            leftControlPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, formulaHeight));

            formulaLabel = new Label
            {
                Text = "Rumus:",
                AutoSize = false,
                Dock = DockStyle.Fill,
                TextAlign = ContentAlignment.TopLeft,
                Padding = new Padding(0, 5, 0, 0)
            };
            leftControlPanel.Controls.Add(formulaLabel, 0, 0);
            leftControlPanel.SetColumnSpan(formulaLabel, 2);

            int currentRow = 1;
            foreach (var param in paramConfigs)
            {
                leftControlPanel.RowStyles.Add(new RowStyle(SizeType.Absolute, paramHeight));

                param.ParameterLabel = new Label
                {
                    Text = $"{param.Name}: {param.CurrentValue.ToString(param.ValueFormat)}",
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleLeft,
                    Margin = new Padding(0)
                };
                leftControlPanel.Controls.Add(param.ParameterLabel, 0, currentRow);

                param.ParameterTrackBar = new TrackBar
                {
                    Minimum = param.TrackBarMin,
                    Maximum = param.TrackBarMax,
                    SmallChange = 1,
                    LargeChange = Math.Max(1, (param.TrackBarMax - param.TrackBarMin) / 10),
                    TickFrequency = Math.Max(1, (param.TrackBarMax - param.TrackBarMin) / 10),
                    Value = param.TrackBarDefault,
                    Dock = DockStyle.Fill,
                    Margin = new Padding(0)
                };
                param.ParameterTrackBar.Scroll += scrollHandler;

                leftControlPanel.Controls.Add(param.ParameterTrackBar, 1, currentRow);

                param.UpdateValueFromTrackBar();
                currentRow++;
            }
            leftControlPanel.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));

            TableLayoutPanel rightChartPanel = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(0)
            };
            splitPanel.Controls.Add(rightChartPanel, 1, 0);
            rightChartPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
            rightChartPanel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));

            TableLayoutPanel timeChartContainer = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = 1,
                Margin = new Padding(5, 5, 2, 5)
            };
            timeChartContainer.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            rightChartPanel.Controls.Add(timeChartContainer, 0, 0);

            chartTime = new Chart { Dock = DockStyle.Fill };
            ConfigureChart(chartTime, $"{groupText} - Time", "Time (s)", "Output", isFrequencyChart: false);
            timeChartContainer.Controls.Add(chartTime, 0, 0);

            TableLayoutPanel freqChartContainer = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 1,
                RowCount = (maxFreqInfo != null) ? 2 : 1,
                Margin = new Padding(2, 5, 5, 5)
            };
            if (maxFreqInfo != null)
            {
                freqChartContainer.RowStyles.Add(new RowStyle(SizeType.Percent, 85F));
                freqChartContainer.RowStyles.Add(new RowStyle(SizeType.Percent, 15F));
            }
            else
            {
                freqChartContainer.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            }
            rightChartPanel.Controls.Add(freqChartContainer, 1, 0);

            chartFreq = new Chart { Dock = DockStyle.Fill };
            freqChartContainer.Controls.Add(chartFreq, 0, 0);

            if (maxFreqInfo != null)
            {
                Label maxFreqLabel = new Label
                {
                    Text = maxFreqInfo,
                    Dock = DockStyle.Fill,
                    TextAlign = ContentAlignment.MiddleCenter,
                    Font = new Font(this.Font.FontFamily, 8, FontStyle.Italic),
                    ForeColor = Color.DarkSlateGray,
                    Padding = new Padding(0, 2, 0, 0)
                };
                freqChartContainer.Controls.Add(maxFreqLabel, 0, 1);
            }

            int contentInputHeight = formulaHeight + (paramConfigs.Count * paramHeight) + 20;
            const int desiredChartHeight = 350;

            int finalHeight = Math.Max(contentInputHeight, desiredChartHeight);
            groupBox.Height = finalHeight;
        }

        private void ConfigureChart(Chart chart, string title, string xAxisTitle, string yAxisTitle, bool isFrequencyChart, double xAxisMaxFrequency = 0)
        {
            if (chart.InvokeRequired)
            {
                chart.Invoke(new MethodInvoker(delegate { ConfigureChart(chart, title, xAxisTitle, yAxisTitle, isFrequencyChart, xAxisMaxFrequency); }));
                return;
            }

            chart.Series.Clear();
            chart.ChartAreas.Clear();
            chart.Titles.Clear();

            ChartArea chartArea = new ChartArea();
            chart.ChartAreas.Add(chartArea);

            chartArea.Position.Auto = false;
            chartArea.Position.Height = 94;
            chartArea.Position.Width = 94;
            chartArea.Position.X = 3;
            chartArea.Position.Y = 3;

            chart.Titles.Add(title);
            chart.ChartAreas[0].AxisX.Title = xAxisTitle;
            chart.ChartAreas[0].AxisY.Title = yAxisTitle;
            chart.ChartAreas[0].AxisX.LabelStyle.Format = "{F2}";
            chart.ChartAreas[0].AxisY.LabelStyle.Format = "{F2}";

            if (isFrequencyChart)
            {
                chart.ChartAreas[0].AxisX.IsStartedFromZero = true;
                chart.ChartAreas[0].AxisX.Minimum = 0;
                if (xAxisMaxFrequency > 0)
                {
                    chart.ChartAreas[0].AxisX.Maximum = xAxisMaxFrequency;
                    chart.ChartAreas[0].AxisX.Interval = xAxisMaxFrequency / 4;
                }
                chart.ChartAreas[0].AxisY.IsStartedFromZero = true;
            }
            else
            {
                chart.ChartAreas[0].AxisX.IsStartedFromZero = false;
                chart.ChartAreas[0].AxisY.IsStartedFromZero = false;
            }
        }

        private void UpdatePlot(Chart chart, double[] xData, double[] yData, string seriesName = "Data", SeriesChartType chartType = SeriesChartType.Line, Color? color = null)
        {
            if (chart.InvokeRequired)
            {
                chart.Invoke(new MethodInvoker(delegate { UpdatePlot(chart, xData, yData, seriesName, chartType, color); }));
                return;
            }
            if (!chart.ChartAreas.Any()) chart.ChartAreas.Add(new ChartArea());

            ChartingSeries series;
            if (chart.Series.Any(s => s.Name == seriesName))
            {
                series = chart.Series[seriesName];
                series.Points.Clear();
            }
            else
            {
                series = new ChartingSeries(seriesName);
                chart.Series.Add(series);
            }

            series.ChartType = chartType;
            series.Color = color ?? Color.DodgerBlue;

            for (int i = 0; i < xData.Length; i++) series.Points.AddXY(xData[i], yData[i]);

            chart.ChartAreas[0].RecalculateAxesScale();
            chart.Invalidate();
        }

        private (double[] frequencies, double[] magnitudes) CalculateFFT(double[] timeDomainData, double currentSampleRate)
        {
            int N = timeDomainData.Length;
            int N_fft = 1;
            while (N_fft < N) N_fft <<= 1;

            Complex[] complexData = new Complex[N_fft];
            for (int i = 0; i < N; i++) complexData[i] = new Complex(timeDomainData[i], 0);
            for (int i = N; i < N_fft; i++) complexData[i] = new Complex(0, 0);

            Fourier.Forward(complexData, FourierOptions.Matlab);

            double[] frequencies = new double[N_fft / 2];
            double[] magnitudes = new double[N_fft / 2];

            for (int i = 0; i < N_fft / 2; i++)
            {
                frequencies[i] = i * currentSampleRate / N_fft;
                magnitudes[i] = complexData[i].Magnitude / N_fft;
            }
            return (frequencies, magnitudes);
        }

        // --- Update Logic Sensors ---
        private void tbAcoustic_Scroll(object sender, EventArgs e)
        {
            UpdateAcousticSensor();
            UpdateSystemOverallFormula();
            UpdateSystemPlots();
        }
        private void UpdateAcousticSensor()
        {
            foreach (var param in _acousticParams) param.UpdateValueFromTrackBar();

            double rl_value = RL_ACOUSTIC_CONSTANT;
            double p_engine_amp = _acousticParams.First(p => p.Name == "P_engine").CurrentValue;
            double p_combustion_amp = _acousticParams.First(p => p.Name == "P_combustion").CurrentValue;
            double p_vibration_amp = _acousticParams.First(p => p.Name == "P_vibration").CurrentValue;

            lblAcousticFormula.Text = $"v_out(t) = S * P_total(t) * (RL / (RL + Rs))\n\n" +
                                      $"S = {S_ACOUSTIC:F4} V/Pa (Konstanta)\n" +
                                      $"Rs = {RS_ACOUSTIC:F0} Ω (Konstanta)\n" +
                                      $"RL = {RL_ACOUSTIC_CONSTANT:F0} Ω (Konstanta)\n" +
                                      $"n_sensor(t) = 0 (Konstanta, noise dihilangkan)\n" +
                                      $"P_total(t) = {p_engine_amp:F2}*sin(2π*{ENGINE_FREQ:F0}t) + {p_combustion_amp:F3}*sin(2π*{COMBUSTION_FREQ:F0}t) + {p_vibration_amp:F3}*sin(2π*{VIBRATION_FREQ:F0}t)";

            double currentSensorSampleRate = ACOUSTIC_SAMPLING_RATE;
            int currentNumSamples = (int)(currentSensorSampleRate * SimulationDuration);

            double[] timeData = new double[currentNumSamples];
            double[] vOut = new double[currentNumSamples];
            double timeStep = SimulationDuration / currentNumSamples;

            for (int i = 0; i < currentNumSamples; i++)
            {
                double t = i * timeStep;

                double p_engine_t = p_engine_amp * Math.Sin(2 * Math.PI * ENGINE_FREQ * t);
                double p_combustion_t = p_combustion_amp * Math.Sin(2 * Math.PI * COMBUSTION_FREQ * t);
                double p_vibration_t = p_vibration_amp * Math.Sin(2 * Math.PI * VIBRATION_FREQ * t);

                double p_total_t = p_engine_t + p_combustion_t + p_vibration_t;

                double v_load_t = S_ACOUSTIC * p_total_t * (rl_value / (rl_value + RS_ACOUSTIC));

                vOut[i] = v_load_t;
                timeData[i] = t;
            }
            UpdatePlot(chartAcousticTime, timeData, vOut);

            ConfigureChart(chartAcousticFreq, "Acoustic (AMM-3738-B-R) - Freq", "Frequency (Hz)", "Mag", isFrequencyChart: true, xAxisMaxFrequency: currentSensorSampleRate / 2);
            var fft = CalculateFFT(vOut, currentSensorSampleRate);
            UpdatePlot(chartAcousticFreq, fft.frequencies, fft.magnitudes);
        }

        private void tbAccelerometer_Scroll(object sender, EventArgs e)
        {
            UpdateAccelerometerSensor();
            UpdateSystemOverallFormula();
            UpdateSystemPlots();
        }
        private void UpdateAccelerometerSensor()
        {
            foreach (var param in _accelerometerParams) param.UpdateValueFromTrackBar();

            double a_getaran_amplitude = _accelerometerParams.First(p => p.Name == "a_getaran").CurrentValue;

            lblAccelerometerFormula.Text = $"Vout(t) = V0 + S * a(t)\n\n" +
                                           $"V0 = {V0_ADXL335:F1} V (Konstanta)\n" +
                                           $"S = {S_ADXL335:F3} V/g (Konstanta)\n" +
                                           $"a(t) = {a_getaran_amplitude:F2}g*sin(2π*{ACCELEROMETER_GETARAN_FREQ:F0}t) + {GRAVITY_CONSTANT_VALUE:F1}g";

            double currentSensorSampleRate = ACCELEROMETER_SAMPLING_RATE;
            int currentNumSamples = (int)(currentSensorSampleRate * SimulationDuration);

            double[] timeData = new double[currentNumSamples];
            double[] vOut = new double[currentNumSamples];
            double timeStep = SimulationDuration / currentNumSamples;

            for (int i = 0; i < currentNumSamples; i++)
            {
                double t = i * timeStep;

                double a_getaran_t = a_getaran_amplitude * Math.Sin(2 * Math.PI * ACCELEROMETER_GETARAN_FREQ * t);

                double a_total_t = a_getaran_t + GRAVITY_CONSTANT_VALUE;

                vOut[i] = V0_ADXL335 + S_ADXL335 * a_total_t;
                timeData[i] = t;
            }
            UpdatePlot(chartAccelerometerTime, timeData, vOut);

            ConfigureChart(chartAccelerometerFreq, "Accelerometer (ADXL335) - Freq", "Frequency (Hz)", "Mag", isFrequencyChart: true, xAxisMaxFrequency: currentSensorSampleRate / 2);
            var fft = CalculateFFT(vOut, currentSensorSampleRate);
            UpdatePlot(chartAccelerometerFreq, fft.frequencies, fft.magnitudes);
        }

        private void tbTemperature_Scroll(object sender, EventArgs e)
        {
            UpdateTemperatureSensor();
            UpdateSystemOverallFormula();
            UpdateSystemPlots();
        }
        private void UpdateTemperatureSensor()
        {
            foreach (var param in _temperatureParams) param.UpdateValueFromTrackBar();

            double raw_value = _temperatureParams.First(p => p.Name == "Raw").CurrentValue;

            lblTemperatureFormula.Text = $"T(°C) = raw * LSB\n" +
                                         $"LSB = {LSB_DS18B20:F4}°C (Konstanta)\n\n" +
                                         $"Simulated raw = {raw_value:F0} (Dari input slider)\n" +
                                         $"Raw Range: {RAW_MIN_LIMIT} to {RAW_MAX_LIMIT}";

            double currentSensorSampleRate = DefaultSampleRate;
            int currentNumSamples = (int)(currentSensorSampleRate * SimulationDuration);

            double[] timeData = new double[currentNumSamples];
            double[] temperatureData = new double[currentNumSamples];
            double timeStep = SimulationDuration / currentNumSamples;

            double clamped_raw_value = Math.Min(RAW_MAX_LIMIT, Math.Max(RAW_MIN_LIMIT, raw_value));

            for (int i = 0; i < currentNumSamples; i++)
            {
                timeData[i] = i * timeStep;
                temperatureData[i] = clamped_raw_value * LSB_DS18B20;
            }
            UpdatePlot(chartTemperatureTime, timeData, temperatureData);

            ConfigureChart(chartTemperatureFreq, "Temperature (DS18B20) - Freq", "Frequency (Hz)", "Mag", isFrequencyChart: true, xAxisMaxFrequency: currentSensorSampleRate / 2);
            var fft = CalculateFFT(temperatureData, currentSensorSampleRate);
            UpdatePlot(chartTemperatureFreq, fft.frequencies, fft.magnitudes);
        }

        private void tbOilQuality_Scroll(object sender, EventArgs e)
        {
            UpdateOilQualitySensor();
            UpdateSystemOverallFormula();
            UpdateSystemPlots();
        }
        private void UpdateOilQualitySensor()
        {
            foreach (var param in _oilQualityParams) param.UpdateValueFromTrackBar();

            double e0_value = _oilQualityParams.First(p => p.Name == "Permittivity ε0").CurrentValue;

            lblOilQualityFormula.Text = $"C(t) = ε0 * εr * (A / d)\n\n" +
                                        $"εr = {ER_OIL_QUALITY_CONSTANT:F1} (Konstanta)\n" +
                                        $"A = {A_OIL_QUALITY_CONSTANT:E1} m² (Konstanta)\n" +
                                        $"d = {D_OIL_QUALITY_CONSTANT:E1} m (Konstanta)";

            double currentSensorSampleRate = DefaultSampleRate;
            int currentNumSamples = (int)(currentSensorSampleRate * SimulationDuration);

            double[] timeData = new double[currentNumSamples];
            double[] capacitanceData = new double[currentNumSamples];
            double timeStep = SimulationDuration / currentNumSamples;

            for (int i = 0; i < currentNumSamples; i++)
            {
                double t = i * timeStep;

                capacitanceData[i] = e0_value * ER_OIL_QUALITY_CONSTANT * (A_OIL_QUALITY_CONSTANT / D_OIL_QUALITY_CONSTANT);
                timeData[i] = t;
            }
            UpdatePlot(chartOilQualityTime, timeData, capacitanceData);

            chartOilQualityTime.ChartAreas[0].AxisY.Minimum = 0;
            chartOilQualityTime.ChartAreas[0].AxisY.Maximum = 4e-12;
            chartOilQualityTime.ChartAreas[0].AxisY.Interval = 0.5e-12;
            chartOilQualityTime.ChartAreas[0].AxisY.LabelStyle.Format = "0.0E-00";

            ConfigureChart(chartOilQualityFreq, "Oil Quality (Capacitive probe + AD7745) - Freq", "Frequency (Hz)", "Mag", isFrequencyChart: true, xAxisMaxFrequency: currentSensorSampleRate / 2);
            var fft = CalculateFFT(capacitanceData, currentSensorSampleRate);
            UpdatePlot(chartOilQualityFreq, fft.frequencies, fft.magnitudes);

            chartOilQualityFreq.ChartAreas[0].AxisY.Minimum = 0;
            chartOilQualityFreq.ChartAreas[0].AxisY.Maximum = 4e-15;
            chartOilQualityFreq.ChartAreas[0].AxisY.Interval = 0.5e-15;
            chartOilQualityFreq.ChartAreas[0].AxisY.LabelStyle.Format = "0.0E-00";

            lblOilQualityNotationExplanation.Text =
                $"Simulated ε0: {e0_value:E2} F/m (Dari input slider)\n\n" +
                "Penjelasan notasi ilmiah (E notation):\n" +
                "Angka seperti 5.20E-012 berarti 5.20 kali 10 pangkat -12.\n" +
                "Contoh: 1 pF (pikofarad) = 1E-12 F (Farad)";
        }

        private void tbRpm_Scroll(object sender, EventArgs e)
        {
            UpdateRpmSensor();
            UpdateSystemOverallFormula();
            UpdateSystemPlots();
        }
        private void UpdateRpmSensor()
        {
            foreach (var param in _rpmParams) param.UpdateValueFromTrackBar();

            double count = _rpmParams.First(p => p.Name == "Count").CurrentValue;
            double t_win = _rpmParams.First(p => p.Name == "T_win (s)").CurrentValue;
            double N_pulses_per_rev = _rpmParams.First(p => p.Name == "N (Pulses/Rev)").CurrentValue;

            lblRpmFormula.Text = $"RPM = 60 * (Count / (T_win * N))\n\n" +
                                 $"Konstanta = {RPM_CONVERSION_FACTOR:F0} (Detik ke Menit)\n" +
                                 $"Count = {count:F0} (Jumlah Pulsa)\n" +
                                 $"T_win = {t_win:F1} s (Jendela Pengukuran)\n" +
                                 $"N = {N_pulses_per_rev:F0} Pulsa/Rev";

            double calculatedRpm = 0;
            if (t_win > 0 && N_pulses_per_rev > 0)
            {
                calculatedRpm = RPM_CONVERSION_FACTOR * (count / (t_win * N_pulses_per_rev));
            }

            double currentSensorSampleRate = DefaultSampleRate;
            int currentNumSamples = (int)(currentSensorSampleRate * SimulationDuration);

            double[] timeData = new double[currentNumSamples];
            double[] rpmData = new double[currentNumSamples];

            double timeStep = SimulationDuration / currentNumSamples;

            for (int i = 0; i < currentNumSamples; i++)
            {
                double t = i * timeStep;
                rpmData[i] = calculatedRpm;
                timeData[i] = t;
            }
            UpdatePlot(chartRpmTime, timeData, rpmData);

            ConfigureChart(chartRpmFreq, "RPM (Hall Effect-A3144) - Freq", "Frequency (Hz)", "Mag", isFrequencyChart: true, xAxisMaxFrequency: currentSensorSampleRate / 2);
            var fft = CalculateFFT(rpmData, currentSensorSampleRate);
            UpdatePlot(chartRpmFreq, fft.frequencies, fft.magnitudes);
        }

        private void CalculateDynamicSystemPolesAndZeros()
        {
            _dynamicSPoles.Clear();
            _dynamicZPoles.Clear();
            _dynamicZZeros.Clear();

            double accel_a_getaran_amp = _accelerometerParams.Any() ? _accelerometerParams.First(p => p.Name == "a_getaran").CurrentValue : 0.0;
            double calculatedRpm = 0;
            if (_rpmParams.Any())
            {
                double count = _rpmParams.First(p => p.Name == "Count").CurrentValue;
                double t_win = _rpmParams.First(p => p.Name == "T_win (s)").CurrentValue;
                double N_pulses_per_rev = _rpmParams.First(p => p.Name == "N (Pulses/Rev)").CurrentValue;
                if (t_win > 0 && N_pulses_per_rev > 0)
                {
                    calculatedRpm = RPM_CONVERSION_FACTOR * (count / (t_win * N_pulses_per_rev));
                }
            }

            double e0_value = _oilQualityParams.Any() ? _oilQualityParams.First(p => p.Name == "Permittivity ε0").CurrentValue : (88 * 1e-13);
            double capacitanceValue = e0_value * ER_OIL_QUALITY_CONSTANT * (A_OIL_QUALITY_CONSTANT / D_OIL_QUALITY_CONSTANT);

            // --- Dynamic S-Domain Pole Calculation ---
            double minCapacitance = 2.0e-12;
            double maxCapacitance = 3.5e-12;

            double normalizedCapacitance = Math.Max(0, Math.Min(1, (capacitanceValue - minCapacitance) / (maxCapacitance - minCapacitance)));

            double sigma = -0.2 - (normalizedCapacitance * 1.3);

            double normalizedRpmForFreq = Math.Min(1, calculatedRpm / 3000.0);
            double omega_d = 1.0 + (normalizedRpmForFreq * 2.0);

            double vibrationInfluence = (Math.Abs(accel_a_getaran_amp) / 3.0) * 0.2;
            sigma += vibrationInfluence;

            sigma = Math.Min(-0.1, sigma);

            _dynamicSPoles.Add(new Complex(sigma, omega_d));
            _dynamicSPoles.Add(new Complex(sigma, -omega_d));

            _dynamicSPoles.Add(new Complex(-1.0, 0));
            _dynamicSPoles.Add(new Complex(-0.8, 0.5));
            _dynamicSPoles.Add(new Complex(-0.8, -0.5));


            // --- Transformasi Z-Domain ---
            foreach (var s_pole in _dynamicSPoles)
            {
                _dynamicZPoles.Add(Complex.Exp(s_pole * SYSTEM_SAMPLING_PERIOD_TS));
            }

            _dynamicZZeros.Add(new Complex(0.1, 0));
            _dynamicZZeros.Add(new Complex(-0.5, 0));
        }

        private void UpdateSystemOverallFormula()
        {
            double acoustic_rl_constant = RL_ACOUSTIC_CONSTANT;
            double acoustic_p_engine_amp = _acousticParams.Any() ? _acousticParams.First(p => p.Name == "P_engine").CurrentValue : 0;
            double acoustic_p_combustion_amp = _acousticParams.Any() ? _acousticParams.First(p => p.Name == "P_combustion").CurrentValue : 0;
            double acoustic_p_vibration_amp = _acousticParams.Any() ? _acousticParams.First(p => p.Name == "P_vibration").CurrentValue : 0;

            double accel_a_getaran_amp = _accelerometerParams.Any() ? _accelerometerParams.First(p => p.Name == "a_getaran").CurrentValue : 0;

            double temp_raw_value = _temperatureParams.Any() ? _temperatureParams.First(p => p.Name == "Raw").CurrentValue : 0;

            double oil_e0_value = _oilQualityParams.Any() ? _oilQualityParams.First(p => p.Name == "Permittivity ε0").CurrentValue : 0;

            double rpm_count = _rpmParams.Any() ? _rpmParams.First(p => p.Name == "Count").CurrentValue : 0;
            double rpm_t_win = _rpmParams.Any() ? _rpmParams.First(p => p.Name == "T_win (s)").CurrentValue : 0;
            double rpm_N = _rpmParams.Any() ? _rpmParams.First(p => p.Name == "N (Pulses/Rev)").CurrentValue : 0;

            string formulaText =
                "y(t) = [\n" +
                $"   RPM       = {RPM_CONVERSION_FACTOR:F0} * {rpm_count:F0} / ({rpm_t_win:F1} * {rpm_N:F0})\n" +
                $"   V_Accel   = {V0_ADXL335:F1} + {S_ADXL335:F3} * ({accel_a_getaran_amp:F2}sin(2π{ACCELEROMETER_GETARAN_FREQ:F0}t) + {GRAVITY_CONSTANT_VALUE:F1}g)\n" +
                $"   T_Suhu    = {temp_raw_value:F0} * {LSB_DS18B20:F4}\n" +
                $"   V_Akustik = {S_ACOUSTIC:F4} * ({acoustic_p_engine_amp:F2}sin(2π{ENGINE_FREQ:F0}t) + {acoustic_p_combustion_amp:F3}sin(2π{COMBUSTION_FREQ:F0}t) + {acoustic_p_vibration_amp:F3}sin(2π{VIBRATION_FREQ:F0}t)) * ({acoustic_rl_constant:F0} / ({acoustic_rl_constant:F0} + {RS_ACOUSTIC:F0}))\n" +
                $"   C_Oli     = {oil_e0_value:E2} * {ER_OIL_QUALITY_CONSTANT:F1} * {A_OIL_QUALITY_CONSTANT:E1} / {D_OIL_QUALITY_CONSTANT:E1})\n" +
                "]\n\n" +
                "Keterangan:\n" +
                "y(t) = Vektor keluaran sensor-sensor pada waktu t (nilai di atas adalah ekspresi yang dihitung).\n" +
                "n_total(t) = Noise gabungan semua kanal (dihilangkan dari simulasi ini).\n";

            lblSystemOverallFormula.Text = formulaText;
        }

        private void UpdateSystemPlots()
        {
            CalculateDynamicSystemPolesAndZeros();

            // --- S-DOMAIN CHART ---
            chartSDomain.Series.Clear();
            chartSDomain.Annotations.Clear();

            // 1. Tambahkan Arsiran Daerah Stabil (Left Half Plane)
            RectangleAnnotation stableRegion = new RectangleAnnotation();
            stableRegion.AxisX = chartSDomain.ChartAreas[0].AxisX;
            stableRegion.AxisY = chartSDomain.ChartAreas[0].AxisY;

            // Area: Dari Minimum X ke 0
            stableRegion.X = chartSDomain.ChartAreas[0].AxisX.Minimum;
            stableRegion.Y = chartSDomain.ChartAreas[0].AxisY.Minimum;
            stableRegion.Width = Math.Abs(chartSDomain.ChartAreas[0].AxisX.Minimum);
            stableRegion.Height = chartSDomain.ChartAreas[0].AxisY.Maximum - chartSDomain.ChartAreas[0].AxisY.Minimum;

            // PENEBALAN WARNA ARSIRAN:
            stableRegion.BackColor = Color.FromArgb(150, Color.DarkGray);

            stableRegion.IsSizeAlwaysRelative = false;
            stableRegion.ClipToChartArea = chartSDomain.ChartAreas[0].Name;
            chartSDomain.Annotations.Add(stableRegion);

            // 2. Plot Poles di atas arsiran
            PlotPolesAndZeros(chartSDomain, _dynamicSPoles.ToArray(), null, false);


            // --- Z-DOMAIN CHART ---
            chartZDomain.Series.Clear();
            chartZDomain.Annotations.Clear();
            PlotPolesAndZeros(chartZDomain, _dynamicZPoles.ToArray(), _dynamicZZeros.ToArray(), true);
        }

        private void PlotPolesAndZeros(Chart chart, Complex[] poles, Complex[] zeros, bool isZ)
        {
            if (chart.InvokeRequired) { chart.Invoke(new MethodInvoker(() => PlotPolesAndZeros(chart, poles, zeros, isZ))); return; }

            if (isZ)
            {
                var circle = new ChartingSeries("Unit Circle") { ChartType = SeriesChartType.Line, Color = Color.DarkGray, BorderWidth = 1, IsVisibleInLegend = false };
                for (int i = 0; i <= 360; i += 5) circle.Points.AddXY(Math.Cos(i * Math.PI / 180), Math.Sin(i * Math.PI / 180));
                chart.Series.Add(circle);
            }

            var pSeries = new ChartingSeries("Poles") { ChartType = SeriesChartType.Point, MarkerStyle = MarkerStyle.Cross, MarkerSize = 10, Color = Color.Red, BorderWidth = 2 };
            foreach (var p in poles) pSeries.Points.AddXY(p.Real, p.Imaginary);
            chart.Series.Add(pSeries);

            if (zeros != null && zeros.Length > 0)
            {
                var zSeries = new ChartingSeries("Zeros") { ChartType = SeriesChartType.Point, MarkerStyle = MarkerStyle.Circle, MarkerSize = 8, Color = Color.Green, BorderWidth = 2 };
                foreach (var z in zeros) zSeries.Points.AddXY(z.Real, z.Imaginary);
                chart.Series.Add(zSeries);
            }

            chart.ChartAreas[0].RecalculateAxesScale();
        }
    }
}