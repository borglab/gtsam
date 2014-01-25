namespace Projections
{
    partial class GeocentricPanel
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
            this.m_setButton = new System.Windows.Forms.Button();
            this.m_majorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.m_flatteningTextBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.m_toolTips = new System.Windows.Forms.ToolTip(this.components);
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_altitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_XTextBox = new System.Windows.Forms.TextBox();
            this.m_YTextBox = new System.Windows.Forms.TextBox();
            this.m_ZTextBox = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.m_functionComboBox = new System.Windows.Forms.ComboBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.m_textBox00 = new System.Windows.Forms.TextBox();
            this.m_textBox01 = new System.Windows.Forms.TextBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.m_textBox02 = new System.Windows.Forms.TextBox();
            this.m_textBox12 = new System.Windows.Forms.TextBox();
            this.m_textBox10 = new System.Windows.Forms.TextBox();
            this.m_textBox11 = new System.Windows.Forms.TextBox();
            this.m_textBox22 = new System.Windows.Forms.TextBox();
            this.m_textBox20 = new System.Windows.Forms.TextBox();
            this.m_textBox21 = new System.Windows.Forms.TextBox();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.SuspendLayout();
            //
            // m_setButton
            //
            this.m_setButton.Location = new System.Drawing.Point(12, 107);
            this.m_setButton.Name = "m_setButton";
            this.m_setButton.Size = new System.Drawing.Size(75, 23);
            this.m_setButton.TabIndex = 4;
            this.m_setButton.Text = "Set";
            this.m_toolTips.SetToolTip(this.m_setButton, "Set ellipsoid parameters");
            this.m_setButton.UseVisualStyleBackColor = true;
            this.m_setButton.Click += new System.EventHandler(this.OnSetParameters);
            //
            // m_majorRadiusTextBox
            //
            this.m_majorRadiusTextBox.Location = new System.Drawing.Point(12, 42);
            this.m_majorRadiusTextBox.Name = "m_majorRadiusTextBox";
            this.m_majorRadiusTextBox.Size = new System.Drawing.Size(125, 20);
            this.m_majorRadiusTextBox.TabIndex = 2;
            //
            // groupBox1
            //
            this.groupBox1.Controls.Add(this.m_setButton);
            this.groupBox1.Controls.Add(this.m_flatteningTextBox);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.m_majorRadiusTextBox);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Location = new System.Drawing.Point(7, 7);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(146, 140);
            this.groupBox1.TabIndex = 5;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Ellipsoid Parameters";
            //
            // m_flatteningTextBox
            //
            this.m_flatteningTextBox.Location = new System.Drawing.Point(12, 83);
            this.m_flatteningTextBox.Name = "m_flatteningTextBox";
            this.m_flatteningTextBox.Size = new System.Drawing.Size(125, 20);
            this.m_flatteningTextBox.TabIndex = 3;
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(12, 25);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(109, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Major Radius (meters)";
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(12, 66);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 13);
            this.label2.TabIndex = 1;
            this.label2.Text = "Flattening";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(166, 11);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(95, 13);
            this.label3.TabIndex = 6;
            this.label3.Text = "Latitude ( degrees)";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(166, 38);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(101, 13);
            this.label4.TabIndex = 7;
            this.label4.Text = "Longitude (degrees)";
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(166, 65);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(82, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "Altitude (meters)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(166, 92);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(54, 13);
            this.label6.TabIndex = 9;
            this.label6.Text = "X (meters)";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(166, 119);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(54, 13);
            this.label7.TabIndex = 10;
            this.label7.Text = "Y (meters)";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(166, 146);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(54, 13);
            this.label8.TabIndex = 11;
            this.label8.Text = "Z (meters)";
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(273, 7);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_latitudeTextBox.TabIndex = 12;
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(273, 34);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_longitudeTextBox.TabIndex = 13;
            //
            // m_altitudeTextBox
            //
            this.m_altitudeTextBox.Location = new System.Drawing.Point(273, 61);
            this.m_altitudeTextBox.Name = "m_altitudeTextBox";
            this.m_altitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_altitudeTextBox.TabIndex = 14;
            //
            // m_XTextBox
            //
            this.m_XTextBox.Location = new System.Drawing.Point(273, 88);
            this.m_XTextBox.Name = "m_XTextBox";
            this.m_XTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_XTextBox.TabIndex = 15;
            //
            // m_YTextBox
            //
            this.m_YTextBox.Location = new System.Drawing.Point(273, 115);
            this.m_YTextBox.Name = "m_YTextBox";
            this.m_YTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_YTextBox.TabIndex = 16;
            //
            // m_ZTextBox
            //
            this.m_ZTextBox.Location = new System.Drawing.Point(273, 142);
            this.m_ZTextBox.Name = "m_ZTextBox";
            this.m_ZTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_ZTextBox.TabIndex = 17;
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(169, 175);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(48, 13);
            this.label9.TabIndex = 18;
            this.label9.Text = "Function";
            //
            // m_functionComboBox
            //
            this.m_functionComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_functionComboBox.FormattingEnabled = true;
            this.m_functionComboBox.Items.AddRange(new object[] {
            "Forward",
            "Reverse"});
            this.m_functionComboBox.Location = new System.Drawing.Point(273, 171);
            this.m_functionComboBox.Name = "m_functionComboBox";
            this.m_functionComboBox.Size = new System.Drawing.Size(100, 21);
            this.m_functionComboBox.TabIndex = 19;
            this.m_toolTips.SetToolTip(this.m_functionComboBox, "Forward/Reverse");
            this.m_functionComboBox.SelectedIndexChanged += new System.EventHandler(this.OnFunctionChanged);
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(386, 169);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 20;
            this.button1.Text = "Convert";
            this.m_toolTips.SetToolTip(this.button1, "Converts the coordinates using the selected Function");
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnConvert);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(7, 170);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 21;
            this.button2.Text = "Validate";
            this.m_toolTips.SetToolTip(this.button2, "Tests Geocentric interfaces");
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnValidate);
            //
            // m_textBox00
            //
            this.m_textBox00.Location = new System.Drawing.Point(6, 25);
            this.m_textBox00.Name = "m_textBox00";
            this.m_textBox00.ReadOnly = true;
            this.m_textBox00.Size = new System.Drawing.Size(100, 20);
            this.m_textBox00.TabIndex = 22;
            //
            // m_textBox01
            //
            this.m_textBox01.Location = new System.Drawing.Point(112, 25);
            this.m_textBox01.Name = "m_textBox01";
            this.m_textBox01.ReadOnly = true;
            this.m_textBox01.Size = new System.Drawing.Size(100, 20);
            this.m_textBox01.TabIndex = 23;
            //
            // groupBox2
            //
            this.groupBox2.Controls.Add(this.m_textBox22);
            this.groupBox2.Controls.Add(this.m_textBox20);
            this.groupBox2.Controls.Add(this.m_textBox21);
            this.groupBox2.Controls.Add(this.m_textBox12);
            this.groupBox2.Controls.Add(this.m_textBox10);
            this.groupBox2.Controls.Add(this.m_textBox11);
            this.groupBox2.Controls.Add(this.m_textBox02);
            this.groupBox2.Controls.Add(this.m_textBox00);
            this.groupBox2.Controls.Add(this.m_textBox01);
            this.groupBox2.Location = new System.Drawing.Point(380, 11);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Size = new System.Drawing.Size(325, 99);
            this.groupBox2.TabIndex = 24;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "Rotation Matrix - ENU to ECEF";
            //
            // m_textBox02
            //
            this.m_textBox02.Location = new System.Drawing.Point(218, 25);
            this.m_textBox02.Name = "m_textBox02";
            this.m_textBox02.ReadOnly = true;
            this.m_textBox02.Size = new System.Drawing.Size(100, 20);
            this.m_textBox02.TabIndex = 24;
            //
            // m_textBox12
            //
            this.m_textBox12.Location = new System.Drawing.Point(218, 50);
            this.m_textBox12.Name = "m_textBox12";
            this.m_textBox12.ReadOnly = true;
            this.m_textBox12.Size = new System.Drawing.Size(100, 20);
            this.m_textBox12.TabIndex = 27;
            //
            // m_textBox10
            //
            this.m_textBox10.Location = new System.Drawing.Point(6, 50);
            this.m_textBox10.Name = "m_textBox10";
            this.m_textBox10.ReadOnly = true;
            this.m_textBox10.Size = new System.Drawing.Size(100, 20);
            this.m_textBox10.TabIndex = 25;
            //
            // m_textBox11
            //
            this.m_textBox11.Location = new System.Drawing.Point(112, 50);
            this.m_textBox11.Name = "m_textBox11";
            this.m_textBox11.ReadOnly = true;
            this.m_textBox11.Size = new System.Drawing.Size(100, 20);
            this.m_textBox11.TabIndex = 26;
            //
            // m_textBox22
            //
            this.m_textBox22.Location = new System.Drawing.Point(218, 74);
            this.m_textBox22.Name = "m_textBox22";
            this.m_textBox22.ReadOnly = true;
            this.m_textBox22.Size = new System.Drawing.Size(100, 20);
            this.m_textBox22.TabIndex = 30;
            //
            // m_textBox20
            //
            this.m_textBox20.Location = new System.Drawing.Point(6, 74);
            this.m_textBox20.Name = "m_textBox20";
            this.m_textBox20.ReadOnly = true;
            this.m_textBox20.Size = new System.Drawing.Size(100, 20);
            this.m_textBox20.TabIndex = 28;
            //
            // m_textBox21
            //
            this.m_textBox21.Location = new System.Drawing.Point(112, 74);
            this.m_textBox21.Name = "m_textBox21";
            this.m_textBox21.ReadOnly = true;
            this.m_textBox21.Size = new System.Drawing.Size(100, 20);
            this.m_textBox21.TabIndex = 29;
            //
            // GeocentricPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.groupBox2);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_functionComboBox);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.m_ZTextBox);
            this.Controls.Add(this.m_YTextBox);
            this.Controls.Add(this.m_XTextBox);
            this.Controls.Add(this.m_altitudeTextBox);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.groupBox1);
            this.Name = "GeocentricPanel";
            this.Size = new System.Drawing.Size(771, 231);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox2.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button m_setButton;
        private System.Windows.Forms.ToolTip m_toolTips;
        private System.Windows.Forms.TextBox m_majorRadiusTextBox;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.TextBox m_flatteningTextBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.TextBox m_altitudeTextBox;
        private System.Windows.Forms.TextBox m_XTextBox;
        private System.Windows.Forms.TextBox m_YTextBox;
        private System.Windows.Forms.TextBox m_ZTextBox;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.ComboBox m_functionComboBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.TextBox m_textBox00;
        private System.Windows.Forms.TextBox m_textBox01;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.TextBox m_textBox22;
        private System.Windows.Forms.TextBox m_textBox20;
        private System.Windows.Forms.TextBox m_textBox21;
        private System.Windows.Forms.TextBox m_textBox12;
        private System.Windows.Forms.TextBox m_textBox10;
        private System.Windows.Forms.TextBox m_textBox11;
        private System.Windows.Forms.TextBox m_textBox02;
    }
}
