namespace Projections
{
    partial class AlbersPanel
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
            this.m_originLatitudeTextBox = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.m_centralScaleTextBox = new System.Windows.Forms.TextBox();
            this.m_toolTip = new System.Windows.Forms.ToolTip(this.components);
            this.m_constructorComboBox = new System.Windows.Forms.ComboBox();
            this.m_azimuthalScaleTextBox = new System.Windows.Forms.TextBox();
            this.m_functionComboBox = new System.Windows.Forms.ComboBox();
            this.m_convertButton = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.m_scaleLabel = new System.Windows.Forms.Label();
            this.m_stdLatLabel = new System.Windows.Forms.Label();
            this.m_stdLat2Label = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.m_sinLat2Label = new System.Windows.Forms.Label();
            this.m_cosLat2Label = new System.Windows.Forms.Label();
            this.m_KTextBox = new System.Windows.Forms.TextBox();
            this.m_stdLat1TextBox = new System.Windows.Forms.TextBox();
            this.m_stdLat2TextBox = new System.Windows.Forms.TextBox();
            this.m_sinLat2TextBox = new System.Windows.Forms.TextBox();
            this.m_cosLat2TextBox = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.m_lon0TextBox = new System.Windows.Forms.TextBox();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_xTextBox = new System.Windows.Forms.TextBox();
            this.m_yTextBox = new System.Windows.Forms.TextBox();
            this.m_gammaTextBox = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.label14 = new System.Windows.Forms.Label();
            this.m_projectionComboBox = new System.Windows.Forms.ComboBox();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            //
            // groupBox1
            //
            this.groupBox1.Controls.Add(this.m_setButton);
            this.groupBox1.Controls.Add(this.m_flatteningTextBox);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.m_majorRadiusTextBox);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Location = new System.Drawing.Point(3, 3);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(146, 140);
            this.groupBox1.TabIndex = 6;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Ellipsoid Parameters";
            //
            // m_setButton
            //
            this.m_setButton.Location = new System.Drawing.Point(12, 109);
            this.m_setButton.Name = "m_setButton";
            this.m_setButton.Size = new System.Drawing.Size(75, 23);
            this.m_setButton.TabIndex = 4;
            this.m_setButton.Text = "Set";
            this.m_toolTip.SetToolTip(this.m_setButton, "Set constructor inputs");
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
            this.label3.Location = new System.Drawing.Point(165, 11);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(122, 13);
            this.label3.TabIndex = 7;
            this.label3.Text = "Origin Latitude (degrees)";
            //
            // m_originLatitudeTextBox
            //
            this.m_originLatitudeTextBox.Location = new System.Drawing.Point(319, 7);
            this.m_originLatitudeTextBox.Name = "m_originLatitudeTextBox";
            this.m_originLatitudeTextBox.ReadOnly = true;
            this.m_originLatitudeTextBox.Size = new System.Drawing.Size(115, 20);
            this.m_originLatitudeTextBox.TabIndex = 8;
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(165, 38);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(70, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "Central Scale";
            //
            // m_centralScaleTextBox
            //
            this.m_centralScaleTextBox.Location = new System.Drawing.Point(319, 34);
            this.m_centralScaleTextBox.Name = "m_centralScaleTextBox";
            this.m_centralScaleTextBox.ReadOnly = true;
            this.m_centralScaleTextBox.Size = new System.Drawing.Size(115, 20);
            this.m_centralScaleTextBox.TabIndex = 10;
            //
            // m_constructorComboBox
            //
            this.m_constructorComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_constructorComboBox.FormattingEnabled = true;
            this.m_constructorComboBox.Location = new System.Drawing.Point(10, 170);
            this.m_constructorComboBox.Name = "m_constructorComboBox";
            this.m_constructorComboBox.Size = new System.Drawing.Size(139, 21);
            this.m_constructorComboBox.TabIndex = 18;
            this.m_toolTip.SetToolTip(this.m_constructorComboBox, "Select constructor type");
            this.m_constructorComboBox.SelectedIndexChanged += new System.EventHandler(this.OnConstructorChanged);
            //
            // m_azimuthalScaleTextBox
            //
            this.m_azimuthalScaleTextBox.Location = new System.Drawing.Point(580, 169);
            this.m_azimuthalScaleTextBox.Name = "m_azimuthalScaleTextBox";
            this.m_azimuthalScaleTextBox.ReadOnly = true;
            this.m_azimuthalScaleTextBox.Size = new System.Drawing.Size(109, 20);
            this.m_azimuthalScaleTextBox.TabIndex = 36;
            this.m_toolTip.SetToolTip(this.m_azimuthalScaleTextBox, "Verifies interfaces");
            this.m_azimuthalScaleTextBox.Click += new System.EventHandler(this.OnValidate);
            //
            // m_functionComboBox
            //
            this.m_functionComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_functionComboBox.FormattingEnabled = true;
            this.m_functionComboBox.Items.AddRange(new object[] {
            "Forward",
            "Reverse"});
            this.m_functionComboBox.Location = new System.Drawing.Point(697, 34);
            this.m_functionComboBox.Name = "m_functionComboBox";
            this.m_functionComboBox.Size = new System.Drawing.Size(75, 21);
            this.m_functionComboBox.TabIndex = 38;
            this.m_toolTip.SetToolTip(this.m_functionComboBox, "Select function");
            this.m_functionComboBox.SelectedIndexChanged += new System.EventHandler(this.OnFunction);
            //
            // m_convertButton
            //
            this.m_convertButton.Location = new System.Drawing.Point(697, 114);
            this.m_convertButton.Name = "m_convertButton";
            this.m_convertButton.Size = new System.Drawing.Size(75, 23);
            this.m_convertButton.TabIndex = 39;
            this.m_convertButton.Text = "Convert";
            this.m_toolTip.SetToolTip(this.m_convertButton, "Executes the selected Function using the selected Projection");
            this.m_convertButton.UseVisualStyleBackColor = true;
            this.m_convertButton.Click += new System.EventHandler(this.OnConvert);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(700, 168);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 40;
            this.button2.Text = "Validate";
            this.m_toolTip.SetToolTip(this.button2, "Verifies interfaces");
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnValidate);
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(443, 38);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(92, 13);
            this.label5.TabIndex = 11;
            this.label5.Text = "Latitude (degrees)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(443, 65);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(101, 13);
            this.label6.TabIndex = 12;
            this.label6.Text = "Longitude (degrees)";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(443, 92);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(54, 13);
            this.label7.TabIndex = 13;
            this.label7.Text = "X (meters)";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(443, 119);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(54, 13);
            this.label8.TabIndex = 14;
            this.label8.Text = "Y (meters)";
            //
            // m_scaleLabel
            //
            this.m_scaleLabel.AutoSize = true;
            this.m_scaleLabel.Location = new System.Drawing.Point(165, 65);
            this.m_scaleLabel.Name = "m_scaleLabel";
            this.m_scaleLabel.Size = new System.Drawing.Size(50, 13);
            this.m_scaleLabel.TabIndex = 15;
            this.m_scaleLabel.Text = "Scale (K)";
            //
            // m_stdLatLabel
            //
            this.m_stdLatLabel.AutoSize = true;
            this.m_stdLatLabel.Location = new System.Drawing.Point(165, 92);
            this.m_stdLatLabel.Name = "m_stdLatLabel";
            this.m_stdLatLabel.Size = new System.Drawing.Size(147, 13);
            this.m_stdLatLabel.TabIndex = 16;
            this.m_stdLatLabel.Text = "Standard Latitude 1 (degrees)";
            //
            // m_stdLat2Label
            //
            this.m_stdLat2Label.AutoSize = true;
            this.m_stdLat2Label.Location = new System.Drawing.Point(165, 119);
            this.m_stdLat2Label.Name = "m_stdLat2Label";
            this.m_stdLat2Label.Size = new System.Drawing.Size(147, 13);
            this.m_stdLat2Label.TabIndex = 17;
            this.m_stdLat2Label.Text = "Standard Latitude 2 (degrees)";
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(9, 150);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(61, 13);
            this.label12.TabIndex = 19;
            this.label12.Text = "Constructor";
            //
            // m_sinLat2Label
            //
            this.m_sinLat2Label.AutoSize = true;
            this.m_sinLat2Label.Location = new System.Drawing.Point(165, 146);
            this.m_sinLat2Label.Name = "m_sinLat2Label";
            this.m_sinLat2Label.Size = new System.Drawing.Size(49, 13);
            this.m_sinLat2Label.TabIndex = 20;
            this.m_sinLat2Label.Text = "Sin(Lat2)";
            //
            // m_cosLat2Label
            //
            this.m_cosLat2Label.AutoSize = true;
            this.m_cosLat2Label.Location = new System.Drawing.Point(165, 173);
            this.m_cosLat2Label.Name = "m_cosLat2Label";
            this.m_cosLat2Label.Size = new System.Drawing.Size(52, 13);
            this.m_cosLat2Label.TabIndex = 21;
            this.m_cosLat2Label.Text = "Cos(Lat2)";
            //
            // m_KTextBox
            //
            this.m_KTextBox.Location = new System.Drawing.Point(319, 61);
            this.m_KTextBox.Name = "m_KTextBox";
            this.m_KTextBox.Size = new System.Drawing.Size(115, 20);
            this.m_KTextBox.TabIndex = 22;
            //
            // m_stdLat1TextBox
            //
            this.m_stdLat1TextBox.Location = new System.Drawing.Point(319, 88);
            this.m_stdLat1TextBox.Name = "m_stdLat1TextBox";
            this.m_stdLat1TextBox.Size = new System.Drawing.Size(115, 20);
            this.m_stdLat1TextBox.TabIndex = 23;
            //
            // m_stdLat2TextBox
            //
            this.m_stdLat2TextBox.Location = new System.Drawing.Point(319, 115);
            this.m_stdLat2TextBox.Name = "m_stdLat2TextBox";
            this.m_stdLat2TextBox.Size = new System.Drawing.Size(115, 20);
            this.m_stdLat2TextBox.TabIndex = 24;
            //
            // m_sinLat2TextBox
            //
            this.m_sinLat2TextBox.Location = new System.Drawing.Point(319, 142);
            this.m_sinLat2TextBox.Name = "m_sinLat2TextBox";
            this.m_sinLat2TextBox.Size = new System.Drawing.Size(115, 20);
            this.m_sinLat2TextBox.TabIndex = 25;
            //
            // m_cosLat2TextBox
            //
            this.m_cosLat2TextBox.Location = new System.Drawing.Point(319, 169);
            this.m_cosLat2TextBox.Name = "m_cosLat2TextBox";
            this.m_cosLat2TextBox.Size = new System.Drawing.Size(115, 20);
            this.m_cosLat2TextBox.TabIndex = 26;
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(443, 11);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(131, 13);
            this.label9.TabIndex = 27;
            this.label9.Text = "Origin Longitude (degrees)";
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(443, 146);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(90, 13);
            this.label10.TabIndex = 28;
            this.label10.Text = "Gamma (degrees)";
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(443, 173);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(82, 13);
            this.label11.TabIndex = 29;
            this.label11.Text = "Azimuthal Scale";
            //
            // m_lon0TextBox
            //
            this.m_lon0TextBox.Location = new System.Drawing.Point(580, 7);
            this.m_lon0TextBox.Name = "m_lon0TextBox";
            this.m_lon0TextBox.Size = new System.Drawing.Size(109, 20);
            this.m_lon0TextBox.TabIndex = 30;
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(580, 34);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(109, 20);
            this.m_latitudeTextBox.TabIndex = 31;
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(580, 61);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(109, 20);
            this.m_longitudeTextBox.TabIndex = 32;
            //
            // m_xTextBox
            //
            this.m_xTextBox.Location = new System.Drawing.Point(580, 88);
            this.m_xTextBox.Name = "m_xTextBox";
            this.m_xTextBox.Size = new System.Drawing.Size(109, 20);
            this.m_xTextBox.TabIndex = 33;
            //
            // m_yTextBox
            //
            this.m_yTextBox.Location = new System.Drawing.Point(580, 115);
            this.m_yTextBox.Name = "m_yTextBox";
            this.m_yTextBox.Size = new System.Drawing.Size(109, 20);
            this.m_yTextBox.TabIndex = 34;
            //
            // m_gammaTextBox
            //
            this.m_gammaTextBox.Location = new System.Drawing.Point(580, 142);
            this.m_gammaTextBox.Name = "m_gammaTextBox";
            this.m_gammaTextBox.ReadOnly = true;
            this.m_gammaTextBox.Size = new System.Drawing.Size(109, 20);
            this.m_gammaTextBox.TabIndex = 35;
            //
            // label13
            //
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(697, 11);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(48, 13);
            this.label13.TabIndex = 37;
            this.label13.Text = "Function";
            //
            // label14
            //
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(697, 65);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(54, 13);
            this.label14.TabIndex = 41;
            this.label14.Text = "Projection";
            //
            // m_projectionComboBox
            //
            this.m_projectionComboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_projectionComboBox.FormattingEnabled = true;
            this.m_projectionComboBox.Items.AddRange(new object[] {
            "Albers Equal Area",
            "Lambert Conformal Conic",
            "Transverse Mercator",
            "Transverse Mercator Exact"});
            this.m_projectionComboBox.Location = new System.Drawing.Point(697, 88);
            this.m_projectionComboBox.Name = "m_projectionComboBox";
            this.m_projectionComboBox.Size = new System.Drawing.Size(134, 21);
            this.m_projectionComboBox.TabIndex = 42;
            this.m_toolTip.SetToolTip(this.m_projectionComboBox, "Projection Type");
            this.m_projectionComboBox.SelectedIndexChanged += new System.EventHandler(this.OnProjection);
            //
            // AlbersPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_projectionComboBox);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.m_convertButton);
            this.Controls.Add(this.m_functionComboBox);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.m_azimuthalScaleTextBox);
            this.Controls.Add(this.m_gammaTextBox);
            this.Controls.Add(this.m_yTextBox);
            this.Controls.Add(this.m_xTextBox);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.m_lon0TextBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.m_cosLat2TextBox);
            this.Controls.Add(this.m_sinLat2TextBox);
            this.Controls.Add(this.m_stdLat2TextBox);
            this.Controls.Add(this.m_stdLat1TextBox);
            this.Controls.Add(this.m_KTextBox);
            this.Controls.Add(this.m_cosLat2Label);
            this.Controls.Add(this.m_sinLat2Label);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.m_constructorComboBox);
            this.Controls.Add(this.m_stdLat2Label);
            this.Controls.Add(this.m_stdLatLabel);
            this.Controls.Add(this.m_scaleLabel);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.m_centralScaleTextBox);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.m_originLatitudeTextBox);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.groupBox1);
            this.Name = "AlbersPanel";
            this.Size = new System.Drawing.Size(851, 248);
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
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox m_originLatitudeTextBox;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox m_centralScaleTextBox;
        private System.Windows.Forms.ToolTip m_toolTip;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label m_scaleLabel;
        private System.Windows.Forms.Label m_stdLatLabel;
        private System.Windows.Forms.Label m_stdLat2Label;
        private System.Windows.Forms.ComboBox m_constructorComboBox;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label m_sinLat2Label;
        private System.Windows.Forms.Label m_cosLat2Label;
        private System.Windows.Forms.TextBox m_KTextBox;
        private System.Windows.Forms.TextBox m_stdLat1TextBox;
        private System.Windows.Forms.TextBox m_stdLat2TextBox;
        private System.Windows.Forms.TextBox m_sinLat2TextBox;
        private System.Windows.Forms.TextBox m_cosLat2TextBox;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox m_lon0TextBox;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.TextBox m_xTextBox;
        private System.Windows.Forms.TextBox m_yTextBox;
        private System.Windows.Forms.TextBox m_gammaTextBox;
        private System.Windows.Forms.TextBox m_azimuthalScaleTextBox;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.ComboBox m_functionComboBox;
        private System.Windows.Forms.Button m_convertButton;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.ComboBox m_projectionComboBox;
    }
}
