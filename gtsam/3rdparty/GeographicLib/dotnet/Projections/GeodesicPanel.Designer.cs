namespace Projections
{
    partial class GeodesicPanel
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
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.m_majorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.m_flatteningTextBox = new System.Windows.Forms.TextBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.m_setButton = new System.Windows.Forms.Button();
            this.button1 = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.m_distanceRadioButton = new System.Windows.Forms.RadioButton();
            this.m_arcLengthRadioButton = new System.Windows.Forms.RadioButton();
            this.m_originLatitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_originLongitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_originAzimuthTextBox = new System.Windows.Forms.TextBox();
            this.m_distanceTextBox = new System.Windows.Forms.TextBox();
            this.m_ArcLengthTextBox = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.m12Label = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.m_finalLatitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_finalLongitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_finalAzimuthTextBox = new System.Windows.Forms.TextBox();
            this.m_reducedLengthTextBox = new System.Windows.Forms.TextBox();
            this.m_M12TextBox = new System.Windows.Forms.TextBox();
            this.m_M21TextBox = new System.Windows.Forms.TextBox();
            this.m_S12TextBox = new System.Windows.Forms.TextBox();
            this.m_functionComboBox = new System.Windows.Forms.ComboBox();
            this.FunctionLabel = new System.Windows.Forms.Label();
            this.m_classComboBox = new System.Windows.Forms.ComboBox();
            this.label12 = new System.Windows.Forms.Label();
            this.m_tooltips = new System.Windows.Forms.ToolTip(this.components);
            this.m_validateButton = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(14, 25);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(109, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Major Radius (meters)";
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(14, 51);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(53, 13);
            this.label2.TabIndex = 1;
            this.label2.Text = "Flattening";
            //
            // m_majorRadiusTextBox
            //
            this.m_majorRadiusTextBox.Location = new System.Drawing.Point(129, 21);
            this.m_majorRadiusTextBox.Name = "m_majorRadiusTextBox";
            this.m_majorRadiusTextBox.Size = new System.Drawing.Size(125, 20);
            this.m_majorRadiusTextBox.TabIndex = 2;
            //
            // m_flatteningTextBox
            //
            this.m_flatteningTextBox.Location = new System.Drawing.Point(129, 47);
            this.m_flatteningTextBox.Name = "m_flatteningTextBox";
            this.m_flatteningTextBox.Size = new System.Drawing.Size(125, 20);
            this.m_flatteningTextBox.TabIndex = 3;
            //
            // groupBox1
            //
            this.groupBox1.Controls.Add(this.m_setButton);
            this.groupBox1.Controls.Add(this.m_flatteningTextBox);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.m_majorRadiusTextBox);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Location = new System.Drawing.Point(4, 4);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(267, 107);
            this.groupBox1.TabIndex = 4;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Ellipsoid Parameters";
            //
            // m_setButton
            //
            this.m_setButton.Location = new System.Drawing.Point(176, 74);
            this.m_setButton.Name = "m_setButton";
            this.m_setButton.Size = new System.Drawing.Size(75, 23);
            this.m_setButton.TabIndex = 4;
            this.m_setButton.Text = "Set";
            this.m_setButton.UseVisualStyleBackColor = true;
            this.m_setButton.Click += new System.EventHandler(this.OnSet);
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(183, 143);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 5;
            this.button1.Text = "Execute";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Enter += new System.EventHandler(this.OnForward);
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(279, 14);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(122, 13);
            this.label3.TabIndex = 6;
            this.label3.Text = "Origin Latitude (degrees)";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(279, 41);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(131, 13);
            this.label4.TabIndex = 7;
            this.label4.Text = "Origin Longitude (degrees)";
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(279, 68);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(121, 13);
            this.label5.TabIndex = 8;
            this.label5.Text = "Origin Azimuth (degrees)";
            //
            // m_distanceRadioButton
            //
            this.m_distanceRadioButton.AutoSize = true;
            this.m_distanceRadioButton.Checked = true;
            this.m_distanceRadioButton.Location = new System.Drawing.Point(279, 93);
            this.m_distanceRadioButton.Name = "m_distanceRadioButton";
            this.m_distanceRadioButton.Size = new System.Drawing.Size(107, 17);
            this.m_distanceRadioButton.TabIndex = 9;
            this.m_distanceRadioButton.TabStop = true;
            this.m_distanceRadioButton.Text = "Distance (meters)";
            this.m_distanceRadioButton.UseVisualStyleBackColor = true;
            this.m_distanceRadioButton.Click += new System.EventHandler(this.OnDistance);
            //
            // m_arcLengthRadioButton
            //
            this.m_arcLengthRadioButton.AutoSize = true;
            this.m_arcLengthRadioButton.Location = new System.Drawing.Point(279, 120);
            this.m_arcLengthRadioButton.Name = "m_arcLengthRadioButton";
            this.m_arcLengthRadioButton.Size = new System.Drawing.Size(124, 17);
            this.m_arcLengthRadioButton.TabIndex = 10;
            this.m_arcLengthRadioButton.Text = "Arc Length (degrees)";
            this.m_arcLengthRadioButton.UseVisualStyleBackColor = true;
            this.m_arcLengthRadioButton.Click += new System.EventHandler(this.OnArcLength);
            //
            // m_originLatitudeTextBox
            //
            this.m_originLatitudeTextBox.Location = new System.Drawing.Point(416, 10);
            this.m_originLatitudeTextBox.Name = "m_originLatitudeTextBox";
            this.m_originLatitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_originLatitudeTextBox.TabIndex = 11;
            //
            // m_originLongitudeTextBox
            //
            this.m_originLongitudeTextBox.Location = new System.Drawing.Point(416, 37);
            this.m_originLongitudeTextBox.Name = "m_originLongitudeTextBox";
            this.m_originLongitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_originLongitudeTextBox.TabIndex = 12;
            //
            // m_originAzimuthTextBox
            //
            this.m_originAzimuthTextBox.Location = new System.Drawing.Point(416, 64);
            this.m_originAzimuthTextBox.Name = "m_originAzimuthTextBox";
            this.m_originAzimuthTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_originAzimuthTextBox.TabIndex = 13;
            //
            // m_distanceTextBox
            //
            this.m_distanceTextBox.Location = new System.Drawing.Point(416, 91);
            this.m_distanceTextBox.Name = "m_distanceTextBox";
            this.m_distanceTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_distanceTextBox.TabIndex = 14;
            //
            // m_ArcLengthTextBox
            //
            this.m_ArcLengthTextBox.Location = new System.Drawing.Point(416, 118);
            this.m_ArcLengthTextBox.Name = "m_ArcLengthTextBox";
            this.m_ArcLengthTextBox.ReadOnly = true;
            this.m_ArcLengthTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_ArcLengthTextBox.TabIndex = 15;
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(525, 14);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(117, 13);
            this.label6.TabIndex = 16;
            this.label6.Text = "Final Latitude (degrees)";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(525, 41);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(126, 13);
            this.label7.TabIndex = 17;
            this.label7.Text = "Final Longitude (degrees)";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(525, 68);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(116, 13);
            this.label8.TabIndex = 18;
            this.label8.Text = "Final Azimuth (degrees)";
            //
            // m12Label
            //
            this.m12Label.AutoSize = true;
            this.m12Label.Location = new System.Drawing.Point(525, 95);
            this.m12Label.Name = "m12Label";
            this.m12Label.Size = new System.Drawing.Size(127, 13);
            this.m12Label.TabIndex = 19;
            this.m12Label.Text = "Reduced Length (meters)";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(525, 122);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(28, 13);
            this.label9.TabIndex = 20;
            this.label9.Text = "M12";
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(525, 149);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(28, 13);
            this.label10.TabIndex = 21;
            this.label10.Text = "M21";
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(525, 176);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(26, 13);
            this.label11.TabIndex = 22;
            this.label11.Text = "S12";
            //
            // m_finalLatitudeTextBox
            //
            this.m_finalLatitudeTextBox.Location = new System.Drawing.Point(660, 10);
            this.m_finalLatitudeTextBox.Name = "m_finalLatitudeTextBox";
            this.m_finalLatitudeTextBox.ReadOnly = true;
            this.m_finalLatitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_finalLatitudeTextBox.TabIndex = 23;
            //
            // m_finalLongitudeTextBox
            //
            this.m_finalLongitudeTextBox.Location = new System.Drawing.Point(660, 37);
            this.m_finalLongitudeTextBox.Name = "m_finalLongitudeTextBox";
            this.m_finalLongitudeTextBox.ReadOnly = true;
            this.m_finalLongitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_finalLongitudeTextBox.TabIndex = 24;
            //
            // m_finalAzimuthTextBox
            //
            this.m_finalAzimuthTextBox.Location = new System.Drawing.Point(660, 64);
            this.m_finalAzimuthTextBox.Name = "m_finalAzimuthTextBox";
            this.m_finalAzimuthTextBox.ReadOnly = true;
            this.m_finalAzimuthTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_finalAzimuthTextBox.TabIndex = 25;
            //
            // m_reducedLengthTextBox
            //
            this.m_reducedLengthTextBox.Location = new System.Drawing.Point(660, 91);
            this.m_reducedLengthTextBox.Name = "m_reducedLengthTextBox";
            this.m_reducedLengthTextBox.ReadOnly = true;
            this.m_reducedLengthTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_reducedLengthTextBox.TabIndex = 26;
            //
            // m_M12TextBox
            //
            this.m_M12TextBox.Location = new System.Drawing.Point(660, 118);
            this.m_M12TextBox.Name = "m_M12TextBox";
            this.m_M12TextBox.ReadOnly = true;
            this.m_M12TextBox.Size = new System.Drawing.Size(100, 20);
            this.m_M12TextBox.TabIndex = 27;
            //
            // m_M21TextBox
            //
            this.m_M21TextBox.Location = new System.Drawing.Point(660, 145);
            this.m_M21TextBox.Name = "m_M21TextBox";
            this.m_M21TextBox.ReadOnly = true;
            this.m_M21TextBox.Size = new System.Drawing.Size(100, 20);
            this.m_M21TextBox.TabIndex = 28;
            //
            // m_S12TextBox
            //
            this.m_S12TextBox.Location = new System.Drawing.Point(660, 172);
            this.m_S12TextBox.Name = "m_S12TextBox";
            this.m_S12TextBox.ReadOnly = true;
            this.m_S12TextBox.Size = new System.Drawing.Size(100, 20);
            this.m_S12TextBox.TabIndex = 29;
            //
            // m_functionComboBox
            //
            this.m_functionComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_functionComboBox.FormattingEnabled = true;
            this.m_functionComboBox.Items.AddRange(new object[] {
            "Direct",
            "Inverse"});
            this.m_functionComboBox.Location = new System.Drawing.Point(416, 145);
            this.m_functionComboBox.Name = "m_functionComboBox";
            this.m_functionComboBox.Size = new System.Drawing.Size(100, 21);
            this.m_functionComboBox.TabIndex = 30;
            this.m_functionComboBox.SelectedIndexChanged += new System.EventHandler(this.OnFunction);
            //
            // FunctionLabel
            //
            this.FunctionLabel.AutoSize = true;
            this.FunctionLabel.Location = new System.Drawing.Point(282, 149);
            this.FunctionLabel.Name = "FunctionLabel";
            this.FunctionLabel.Size = new System.Drawing.Size(48, 13);
            this.FunctionLabel.TabIndex = 31;
            this.FunctionLabel.Text = "Function";
            //
            // m_classComboBox
            //
            this.m_classComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_classComboBox.FormattingEnabled = true;
            this.m_classComboBox.Items.AddRange(new object[] {
            "Geodesic",
            "GeodesicExact",
            "GeodesicLine",
            "GeodesicLineExact"});
            this.m_classComboBox.Location = new System.Drawing.Point(395, 172);
            this.m_classComboBox.Name = "m_classComboBox";
            this.m_classComboBox.Size = new System.Drawing.Size(121, 21);
            this.m_classComboBox.TabIndex = 32;
            this.m_classComboBox.SelectedIndexChanged += new System.EventHandler(this.OnClassChanged);
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(282, 176);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(32, 13);
            this.label12.TabIndex = 33;
            this.label12.Text = "Class";
            //
            // m_validateButton
            //
            this.m_validateButton.Location = new System.Drawing.Point(21, 143);
            this.m_validateButton.Name = "m_validateButton";
            this.m_validateButton.Size = new System.Drawing.Size(75, 23);
            this.m_validateButton.TabIndex = 34;
            this.m_validateButton.Text = "Validate";
            this.m_validateButton.UseVisualStyleBackColor = true;
            this.m_validateButton.Click += new System.EventHandler(this.OnValidate);
            //
            // GeodesicPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_validateButton);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.m_classComboBox);
            this.Controls.Add(this.FunctionLabel);
            this.Controls.Add(this.m_functionComboBox);
            this.Controls.Add(this.m_S12TextBox);
            this.Controls.Add(this.m_M21TextBox);
            this.Controls.Add(this.m_M12TextBox);
            this.Controls.Add(this.m_reducedLengthTextBox);
            this.Controls.Add(this.m_finalAzimuthTextBox);
            this.Controls.Add(this.m_finalLongitudeTextBox);
            this.Controls.Add(this.m_finalLatitudeTextBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.m12Label);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.m_ArcLengthTextBox);
            this.Controls.Add(this.m_distanceTextBox);
            this.Controls.Add(this.m_originAzimuthTextBox);
            this.Controls.Add(this.m_originLongitudeTextBox);
            this.Controls.Add(this.m_originLatitudeTextBox);
            this.Controls.Add(this.m_arcLengthRadioButton);
            this.Controls.Add(this.m_distanceRadioButton);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.groupBox1);
            this.Name = "GeodesicPanel";
            this.Size = new System.Drawing.Size(794, 220);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox m_majorRadiusTextBox;
        private System.Windows.Forms.TextBox m_flatteningTextBox;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button m_setButton;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.RadioButton m_distanceRadioButton;
        private System.Windows.Forms.RadioButton m_arcLengthRadioButton;
        private System.Windows.Forms.TextBox m_originLatitudeTextBox;
        private System.Windows.Forms.TextBox m_originLongitudeTextBox;
        private System.Windows.Forms.TextBox m_originAzimuthTextBox;
        private System.Windows.Forms.TextBox m_distanceTextBox;
        private System.Windows.Forms.TextBox m_ArcLengthTextBox;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label m12Label;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox m_finalLatitudeTextBox;
        private System.Windows.Forms.TextBox m_finalLongitudeTextBox;
        private System.Windows.Forms.TextBox m_finalAzimuthTextBox;
        private System.Windows.Forms.TextBox m_reducedLengthTextBox;
        private System.Windows.Forms.TextBox m_M12TextBox;
        private System.Windows.Forms.TextBox m_M21TextBox;
        private System.Windows.Forms.TextBox m_S12TextBox;
        private System.Windows.Forms.ComboBox m_functionComboBox;
        private System.Windows.Forms.Label FunctionLabel;
        private System.Windows.Forms.ComboBox m_classComboBox;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.ToolTip m_tooltips;
        private System.Windows.Forms.Button m_validateButton;
    }
}
