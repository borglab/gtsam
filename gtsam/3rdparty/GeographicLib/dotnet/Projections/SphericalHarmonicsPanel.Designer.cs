namespace Projections
{
    partial class SphericalHarmonicsPanel
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Component Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.m_toolTip = new System.Windows.Forms.ToolTip(this.components);
            this.m_classComboBox = new System.Windows.Forms.ComboBox();
            this.button1 = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.m_tau1Label = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.m_xTextBox = new System.Windows.Forms.TextBox();
            this.m_yTextBox = new System.Windows.Forms.TextBox();
            this.m_zTextBox = new System.Windows.Forms.TextBox();
            this.m_tau1TextBox = new System.Windows.Forms.TextBox();
            this.m_tau2TextBox = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.m_sumTextBox = new System.Windows.Forms.TextBox();
            this.m_gradXTextBox = new System.Windows.Forms.TextBox();
            this.m_gradYTextBox = new System.Windows.Forms.TextBox();
            this.m_gradZTextBox = new System.Windows.Forms.TextBox();
            this.button2 = new System.Windows.Forms.Button();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.m_circleRadiusTextBox = new System.Windows.Forms.TextBox();
            this.m_circleHeightTextBox = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.button3 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            //
            // m_classComboBox
            //
            this.m_classComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_classComboBox.FormattingEnabled = true;
            this.m_classComboBox.Items.AddRange(new object[] {
            "Spherical Harmonic",
            "Spherical Harmonic 1",
            "Spherical Harmonic 2"});
            this.m_classComboBox.Location = new System.Drawing.Point(4, 155);
            this.m_classComboBox.Name = "m_classComboBox";
            this.m_classComboBox.Size = new System.Drawing.Size(139, 21);
            this.m_classComboBox.TabIndex = 10;
            this.m_toolTip.SetToolTip(this.m_classComboBox, "Select Class");
            this.m_classComboBox.SelectedIndexChanged += new System.EventHandler(this.OnClass);
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(203, 112);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 12;
            this.button1.Text = "Compute";
            this.m_toolTip.SetToolTip(this.button1, "Compute harmonic sum and gradient");
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnCompute);
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(4, 8);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(14, 13);
            this.label2.TabIndex = 0;
            this.label2.Text = "X";
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(4, 34);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(14, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Y";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(4, 60);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(14, 13);
            this.label3.TabIndex = 2;
            this.label3.Text = "Z";
            //
            // m_tau1Label
            //
            this.m_tau1Label.AutoSize = true;
            this.m_tau1Label.Location = new System.Drawing.Point(4, 86);
            this.m_tau1Label.Name = "m_tau1Label";
            this.m_tau1Label.Size = new System.Drawing.Size(31, 13);
            this.m_tau1Label.TabIndex = 3;
            this.m_tau1Label.Text = "tau 1";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(4, 112);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(31, 13);
            this.label4.TabIndex = 4;
            this.label4.Text = "tau 2";
            //
            // m_xTextBox
            //
            this.m_xTextBox.Location = new System.Drawing.Point(43, 4);
            this.m_xTextBox.Name = "m_xTextBox";
            this.m_xTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_xTextBox.TabIndex = 5;
            //
            // m_yTextBox
            //
            this.m_yTextBox.Location = new System.Drawing.Point(43, 30);
            this.m_yTextBox.Name = "m_yTextBox";
            this.m_yTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_yTextBox.TabIndex = 6;
            //
            // m_zTextBox
            //
            this.m_zTextBox.Location = new System.Drawing.Point(43, 56);
            this.m_zTextBox.Name = "m_zTextBox";
            this.m_zTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_zTextBox.TabIndex = 7;
            //
            // m_tau1TextBox
            //
            this.m_tau1TextBox.Location = new System.Drawing.Point(43, 82);
            this.m_tau1TextBox.Name = "m_tau1TextBox";
            this.m_tau1TextBox.Size = new System.Drawing.Size(100, 20);
            this.m_tau1TextBox.TabIndex = 8;
            //
            // m_tau2TextBox
            //
            this.m_tau2TextBox.Location = new System.Drawing.Point(43, 108);
            this.m_tau2TextBox.Name = "m_tau2TextBox";
            this.m_tau2TextBox.Size = new System.Drawing.Size(100, 20);
            this.m_tau2TextBox.TabIndex = 9;
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(4, 136);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(32, 13);
            this.label5.TabIndex = 11;
            this.label5.Text = "Class";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(158, 8);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(76, 13);
            this.label6.TabIndex = 13;
            this.label6.Text = "Harmonic Sum";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(158, 33);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(57, 13);
            this.label7.TabIndex = 14;
            this.label7.Text = "X Gradient";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(158, 60);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(57, 13);
            this.label8.TabIndex = 15;
            this.label8.Text = "Y Gradient";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(158, 86);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(57, 13);
            this.label9.TabIndex = 16;
            this.label9.Text = "X Gradient";
            //
            // m_sumTextBox
            //
            this.m_sumTextBox.Location = new System.Drawing.Point(241, 4);
            this.m_sumTextBox.Name = "m_sumTextBox";
            this.m_sumTextBox.ReadOnly = true;
            this.m_sumTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_sumTextBox.TabIndex = 17;
            //
            // m_gradXTextBox
            //
            this.m_gradXTextBox.Location = new System.Drawing.Point(241, 29);
            this.m_gradXTextBox.Name = "m_gradXTextBox";
            this.m_gradXTextBox.ReadOnly = true;
            this.m_gradXTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_gradXTextBox.TabIndex = 18;
            //
            // m_gradYTextBox
            //
            this.m_gradYTextBox.Location = new System.Drawing.Point(241, 56);
            this.m_gradYTextBox.Name = "m_gradYTextBox";
            this.m_gradYTextBox.ReadOnly = true;
            this.m_gradYTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_gradYTextBox.TabIndex = 19;
            //
            // m_gradZTextBox
            //
            this.m_gradZTextBox.Location = new System.Drawing.Point(241, 82);
            this.m_gradZTextBox.Name = "m_gradZTextBox";
            this.m_gradZTextBox.ReadOnly = true;
            this.m_gradZTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_gradZTextBox.TabIndex = 20;
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(406, 86);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(100, 23);
            this.button2.TabIndex = 21;
            this.button2.Text = "Circular Engine";
            this.m_toolTip.SetToolTip(this.button2, "Calculates the sum for the given radius, height, and longitude using a Circular E" +
                    "ngine");
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnCircularEngine);
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(348, 8);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(69, 13);
            this.label10.TabIndex = 22;
            this.label10.Text = "Circle Radius";
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(348, 33);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(67, 13);
            this.label11.TabIndex = 23;
            this.label11.Text = "Circle Height";
            //
            // m_circleRadiusTextBox
            //
            this.m_circleRadiusTextBox.Location = new System.Drawing.Point(458, 4);
            this.m_circleRadiusTextBox.Name = "m_circleRadiusTextBox";
            this.m_circleRadiusTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_circleRadiusTextBox.TabIndex = 24;
            //
            // m_circleHeightTextBox
            //
            this.m_circleHeightTextBox.Location = new System.Drawing.Point(458, 29);
            this.m_circleHeightTextBox.Name = "m_circleHeightTextBox";
            this.m_circleHeightTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_circleHeightTextBox.TabIndex = 25;
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(348, 60);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(101, 13);
            this.label12.TabIndex = 26;
            this.label12.Text = "Longitude (degrees)";
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(458, 56);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_longitudeTextBox.TabIndex = 27;
            //
            // button3
            //
            this.button3.Location = new System.Drawing.Point(203, 155);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 28;
            this.button3.Text = "Validate";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.OnValidate);
            //
            // SphericalHarmonicsPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.button3);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.m_circleHeightTextBox);
            this.Controls.Add(this.m_circleRadiusTextBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.m_gradZTextBox);
            this.Controls.Add(this.m_gradYTextBox);
            this.Controls.Add(this.m_gradXTextBox);
            this.Controls.Add(this.m_sumTextBox);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.m_classComboBox);
            this.Controls.Add(this.m_tau2TextBox);
            this.Controls.Add(this.m_tau1TextBox);
            this.Controls.Add(this.m_zTextBox);
            this.Controls.Add(this.m_yTextBox);
            this.Controls.Add(this.m_xTextBox);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.m_tau1Label);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.label2);
            this.Name = "SphericalHarmonicsPanel";
            this.Size = new System.Drawing.Size(785, 293);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ToolTip m_toolTip;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label m_tau1Label;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox m_xTextBox;
        private System.Windows.Forms.TextBox m_yTextBox;
        private System.Windows.Forms.TextBox m_zTextBox;
        private System.Windows.Forms.TextBox m_tau1TextBox;
        private System.Windows.Forms.TextBox m_tau2TextBox;
        private System.Windows.Forms.ComboBox m_classComboBox;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox m_sumTextBox;
        private System.Windows.Forms.TextBox m_gradXTextBox;
        private System.Windows.Forms.TextBox m_gradYTextBox;
        private System.Windows.Forms.TextBox m_gradZTextBox;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox m_circleRadiusTextBox;
        private System.Windows.Forms.TextBox m_circleHeightTextBox;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.Button button3;
    }
}
