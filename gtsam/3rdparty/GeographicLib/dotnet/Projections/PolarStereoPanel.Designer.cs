namespace Projections
{
    partial class PolarStereoPanel
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
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.m_setButton = new System.Windows.Forms.Button();
            this.m_flatteningTextBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.m_majorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.m_scaleTextBox = new System.Windows.Forms.TextBox();
            this.m_toolTip = new System.Windows.Forms.ToolTip(this.components);
            this.m_northPoleCheckBox = new System.Windows.Forms.CheckBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_xTextBox = new System.Windows.Forms.TextBox();
            this.m_yTextBox = new System.Windows.Forms.TextBox();
            this.m_gammaTextBox = new System.Windows.Forms.TextBox();
            this.m_kTextBox = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.m_functionComboBox = new System.Windows.Forms.ComboBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            //
            // groupBox1
            //
            this.groupBox1.Controls.Add(this.m_scaleTextBox);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.m_setButton);
            this.groupBox1.Controls.Add(this.m_flatteningTextBox);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.m_majorRadiusTextBox);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Location = new System.Drawing.Point(3, 3);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(146, 184);
            this.groupBox1.TabIndex = 20;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Ellipsoid Parameters";
            //
            // m_setButton
            //
            this.m_setButton.Location = new System.Drawing.Point(15, 153);
            this.m_setButton.Name = "m_setButton";
            this.m_setButton.Size = new System.Drawing.Size(75, 23);
            this.m_setButton.TabIndex = 4;
            this.m_setButton.Text = "Set";
            this.m_toolTip.SetToolTip(this.m_setButton, "Sets parameters");
            this.m_setButton.UseVisualStyleBackColor = true;
            this.m_setButton.Click += new System.EventHandler(this.OnSet);
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
            // m_majorRadiusTextBox
            //
            this.m_majorRadiusTextBox.Location = new System.Drawing.Point(12, 42);
            this.m_majorRadiusTextBox.Name = "m_majorRadiusTextBox";
            this.m_majorRadiusTextBox.Size = new System.Drawing.Size(125, 20);
            this.m_majorRadiusTextBox.TabIndex = 2;
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
            this.label3.Location = new System.Drawing.Point(12, 110);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(103, 13);
            this.label3.TabIndex = 5;
            this.label3.Text = "Central Scale Factor";
            //
            // m_scaleTextBox
            //
            this.m_scaleTextBox.Location = new System.Drawing.Point(12, 127);
            this.m_scaleTextBox.Name = "m_scaleTextBox";
            this.m_scaleTextBox.Size = new System.Drawing.Size(125, 20);
            this.m_scaleTextBox.TabIndex = 6;
            //
            // m_northPoleCheckBox
            //
            this.m_northPoleCheckBox.AutoSize = true;
            this.m_northPoleCheckBox.Checked = true;
            this.m_northPoleCheckBox.CheckState = System.Windows.Forms.CheckState.Checked;
            this.m_northPoleCheckBox.Location = new System.Drawing.Point(161, 4);
            this.m_northPoleCheckBox.Name = "m_northPoleCheckBox";
            this.m_northPoleCheckBox.Size = new System.Drawing.Size(76, 17);
            this.m_northPoleCheckBox.TabIndex = 21;
            this.m_northPoleCheckBox.Text = "North Pole";
            this.m_toolTip.SetToolTip(this.m_northPoleCheckBox, "checked = Noth Pole, uncheck = South Pole");
            this.m_northPoleCheckBox.UseVisualStyleBackColor = true;
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(161, 28);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(92, 13);
            this.label4.TabIndex = 22;
            this.label4.Text = "Latitude (degrees)";
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(161, 55);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(101, 13);
            this.label5.TabIndex = 23;
            this.label5.Text = "Longitude (degrees)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(161, 81);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(54, 13);
            this.label6.TabIndex = 24;
            this.label6.Text = "X (meters)";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(161, 107);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(54, 13);
            this.label7.TabIndex = 25;
            this.label7.Text = "Y (meters)";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(161, 133);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(90, 13);
            this.label8.TabIndex = 26;
            this.label8.Text = "Gamma (degrees)";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(161, 159);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(34, 13);
            this.label9.TabIndex = 27;
            this.label9.Text = "Scale";
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(274, 25);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(120, 20);
            this.m_latitudeTextBox.TabIndex = 28;
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(274, 51);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(120, 20);
            this.m_longitudeTextBox.TabIndex = 29;
            //
            // m_xTextBox
            //
            this.m_xTextBox.Location = new System.Drawing.Point(274, 77);
            this.m_xTextBox.Name = "m_xTextBox";
            this.m_xTextBox.Size = new System.Drawing.Size(120, 20);
            this.m_xTextBox.TabIndex = 30;
            //
            // m_yTextBox
            //
            this.m_yTextBox.Location = new System.Drawing.Point(274, 103);
            this.m_yTextBox.Name = "m_yTextBox";
            this.m_yTextBox.Size = new System.Drawing.Size(120, 20);
            this.m_yTextBox.TabIndex = 31;
            //
            // m_gammaTextBox
            //
            this.m_gammaTextBox.Location = new System.Drawing.Point(274, 129);
            this.m_gammaTextBox.Name = "m_gammaTextBox";
            this.m_gammaTextBox.ReadOnly = true;
            this.m_gammaTextBox.Size = new System.Drawing.Size(120, 20);
            this.m_gammaTextBox.TabIndex = 32;
            //
            // m_kTextBox
            //
            this.m_kTextBox.Location = new System.Drawing.Point(274, 155);
            this.m_kTextBox.Name = "m_kTextBox";
            this.m_kTextBox.ReadOnly = true;
            this.m_kTextBox.Size = new System.Drawing.Size(120, 20);
            this.m_kTextBox.TabIndex = 33;
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(411, 4);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(48, 13);
            this.label10.TabIndex = 34;
            this.label10.Text = "Function";
            //
            // m_functionComboBox
            //
            this.m_functionComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_functionComboBox.FormattingEnabled = true;
            this.m_functionComboBox.Items.AddRange(new object[] {
            "Forward",
            "Reverse"});
            this.m_functionComboBox.Location = new System.Drawing.Point(414, 25);
            this.m_functionComboBox.Name = "m_functionComboBox";
            this.m_functionComboBox.Size = new System.Drawing.Size(75, 21);
            this.m_functionComboBox.TabIndex = 35;
            this.m_functionComboBox.SelectedIndexChanged += new System.EventHandler(this.OnFunction);
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(414, 53);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 36;
            this.button1.Text = "Convert";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnConvert);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(414, 155);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 37;
            this.button2.Text = "Validate";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnValidate);
            //
            // PolarStereoPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_functionComboBox);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.m_kTextBox);
            this.Controls.Add(this.m_gammaTextBox);
            this.Controls.Add(this.m_yTextBox);
            this.Controls.Add(this.m_xTextBox);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.m_northPoleCheckBox);
            this.Controls.Add(this.groupBox1);
            this.Name = "PolarStereoPanel";
            this.Size = new System.Drawing.Size(779, 299);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button m_setButton;
        private System.Windows.Forms.TextBox m_flatteningTextBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox m_majorRadiusTextBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox m_scaleTextBox;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.ToolTip m_toolTip;
        private System.Windows.Forms.CheckBox m_northPoleCheckBox;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.TextBox m_xTextBox;
        private System.Windows.Forms.TextBox m_yTextBox;
        private System.Windows.Forms.TextBox m_gammaTextBox;
        private System.Windows.Forms.TextBox m_kTextBox;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.ComboBox m_functionComboBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
    }
}
