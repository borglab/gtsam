namespace Projections
{
    partial class EllipsoidPanel
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
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.m_setButton = new System.Windows.Forms.Button();
            this.m_flatteningTextBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.m_majorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.m_minorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.m_quarterMeridianTextBox = new System.Windows.Forms.TextBox();
            this.m_areaTextBox = new System.Windows.Forms.TextBox();
            this.m_volumeTextBox = new System.Windows.Forms.TextBox();
            this.m_2ndFlatTextBox = new System.Windows.Forms.TextBox();
            this.m_3rdFlatTextBox = new System.Windows.Forms.TextBox();
            this.m_ecc2TextBox = new System.Windows.Forms.TextBox();
            this.m_2ecc2TextBox = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.m_phiTextBox = new System.Windows.Forms.TextBox();
            this.label14 = new System.Windows.Forms.Label();
            this.label15 = new System.Windows.Forms.Label();
            this.label16 = new System.Windows.Forms.Label();
            this.label17 = new System.Windows.Forms.Label();
            this.m_parametericLatTextBox = new System.Windows.Forms.TextBox();
            this.m_geocentricLatTextBox = new System.Windows.Forms.TextBox();
            this.m_rectifyingLatTextBox = new System.Windows.Forms.TextBox();
            this.m_authalicLatTextBox = new System.Windows.Forms.TextBox();
            this.m_conformalTextBox = new System.Windows.Forms.TextBox();
            this.m_isometricLatTextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
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
            this.groupBox1.TabIndex = 7;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Ellipsoid Parameters";
            //
            // m_setButton
            //
            this.m_setButton.Location = new System.Drawing.Point(12, 107);
            this.m_setButton.Name = "m_setButton";
            this.m_setButton.Size = new System.Drawing.Size(75, 23);
            this.m_setButton.TabIndex = 4;
            this.m_setButton.Text = "Set";
            this.m_toolTip.SetToolTip(this.m_setButton, "Sets ellipsoid parameters");
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
            this.label3.Location = new System.Drawing.Point(159, 8);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(69, 13);
            this.label3.TabIndex = 8;
            this.label3.Text = "Minor Radius";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(159, 34);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(85, 13);
            this.label4.TabIndex = 9;
            this.label4.Text = "Quarter Meridian";
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(159, 60);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(29, 13);
            this.label5.TabIndex = 10;
            this.label5.Text = "Area";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(159, 86);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(42, 13);
            this.label6.TabIndex = 11;
            this.label6.Text = "Volume";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(159, 112);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(74, 13);
            this.label7.TabIndex = 12;
            this.label7.Text = "2nd Flattening";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(159, 138);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(71, 13);
            this.label8.TabIndex = 13;
            this.label8.Text = "3rd Flattening";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(159, 164);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(74, 13);
            this.label9.TabIndex = 14;
            this.label9.Text = "Eccentricity^2";
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(159, 190);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(94, 13);
            this.label10.TabIndex = 15;
            this.label10.Text = "2nd eccentricity^2";
            //
            // m_minorRadiusTextBox
            //
            this.m_minorRadiusTextBox.Location = new System.Drawing.Point(259, 4);
            this.m_minorRadiusTextBox.Name = "m_minorRadiusTextBox";
            this.m_minorRadiusTextBox.ReadOnly = true;
            this.m_minorRadiusTextBox.Size = new System.Drawing.Size(134, 20);
            this.m_minorRadiusTextBox.TabIndex = 16;
            //
            // m_quarterMeridianTextBox
            //
            this.m_quarterMeridianTextBox.Location = new System.Drawing.Point(259, 30);
            this.m_quarterMeridianTextBox.Name = "m_quarterMeridianTextBox";
            this.m_quarterMeridianTextBox.ReadOnly = true;
            this.m_quarterMeridianTextBox.Size = new System.Drawing.Size(134, 20);
            this.m_quarterMeridianTextBox.TabIndex = 17;
            //
            // m_areaTextBox
            //
            this.m_areaTextBox.Location = new System.Drawing.Point(259, 56);
            this.m_areaTextBox.Name = "m_areaTextBox";
            this.m_areaTextBox.ReadOnly = true;
            this.m_areaTextBox.Size = new System.Drawing.Size(134, 20);
            this.m_areaTextBox.TabIndex = 18;
            //
            // m_volumeTextBox
            //
            this.m_volumeTextBox.Location = new System.Drawing.Point(259, 82);
            this.m_volumeTextBox.Name = "m_volumeTextBox";
            this.m_volumeTextBox.ReadOnly = true;
            this.m_volumeTextBox.Size = new System.Drawing.Size(134, 20);
            this.m_volumeTextBox.TabIndex = 19;
            //
            // m_2ndFlatTextBox
            //
            this.m_2ndFlatTextBox.Location = new System.Drawing.Point(259, 108);
            this.m_2ndFlatTextBox.Name = "m_2ndFlatTextBox";
            this.m_2ndFlatTextBox.ReadOnly = true;
            this.m_2ndFlatTextBox.Size = new System.Drawing.Size(134, 20);
            this.m_2ndFlatTextBox.TabIndex = 20;
            //
            // m_3rdFlatTextBox
            //
            this.m_3rdFlatTextBox.Location = new System.Drawing.Point(259, 134);
            this.m_3rdFlatTextBox.Name = "m_3rdFlatTextBox";
            this.m_3rdFlatTextBox.ReadOnly = true;
            this.m_3rdFlatTextBox.Size = new System.Drawing.Size(134, 20);
            this.m_3rdFlatTextBox.TabIndex = 21;
            //
            // m_ecc2TextBox
            //
            this.m_ecc2TextBox.Location = new System.Drawing.Point(259, 160);
            this.m_ecc2TextBox.Name = "m_ecc2TextBox";
            this.m_ecc2TextBox.ReadOnly = true;
            this.m_ecc2TextBox.Size = new System.Drawing.Size(134, 20);
            this.m_ecc2TextBox.TabIndex = 22;
            //
            // m_2ecc2TextBox
            //
            this.m_2ecc2TextBox.Location = new System.Drawing.Point(259, 186);
            this.m_2ecc2TextBox.Name = "m_2ecc2TextBox";
            this.m_2ecc2TextBox.ReadOnly = true;
            this.m_2ecc2TextBox.Size = new System.Drawing.Size(134, 20);
            this.m_2ecc2TextBox.TabIndex = 23;
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(402, 8);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(22, 13);
            this.label11.TabIndex = 24;
            this.label11.Text = "Phi";
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(402, 34);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(98, 13);
            this.label12.TabIndex = 25;
            this.label12.Text = "Parametric Latitude";
            //
            // label13
            //
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(402, 60);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(100, 13);
            this.label13.TabIndex = 26;
            this.label13.Text = "Geocentric Latitude";
            //
            // m_phiTextBox
            //
            this.m_phiTextBox.Location = new System.Drawing.Point(512, 4);
            this.m_phiTextBox.Name = "m_phiTextBox";
            this.m_phiTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_phiTextBox.TabIndex = 27;
            //
            // label14
            //
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(402, 86);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(95, 13);
            this.label14.TabIndex = 28;
            this.label14.Text = "Rectifying Latitude";
            //
            // label15
            //
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(402, 112);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(86, 13);
            this.label15.TabIndex = 29;
            this.label15.Text = "Authalic Latitude";
            //
            // label16
            //
            this.label16.AutoSize = true;
            this.label16.Location = new System.Drawing.Point(402, 138);
            this.label16.Name = "label16";
            this.label16.Size = new System.Drawing.Size(95, 13);
            this.label16.TabIndex = 30;
            this.label16.Text = "Conformal Latitude";
            //
            // label17
            //
            this.label17.AutoSize = true;
            this.label17.Location = new System.Drawing.Point(402, 164);
            this.label17.Name = "label17";
            this.label17.Size = new System.Drawing.Size(90, 13);
            this.label17.TabIndex = 31;
            this.label17.Text = "Isometric Latitude";
            //
            // m_parametericLatTextBox
            //
            this.m_parametericLatTextBox.Location = new System.Drawing.Point(512, 30);
            this.m_parametericLatTextBox.Name = "m_parametericLatTextBox";
            this.m_parametericLatTextBox.ReadOnly = true;
            this.m_parametericLatTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_parametericLatTextBox.TabIndex = 32;
            //
            // m_geocentricLatTextBox
            //
            this.m_geocentricLatTextBox.Location = new System.Drawing.Point(512, 56);
            this.m_geocentricLatTextBox.Name = "m_geocentricLatTextBox";
            this.m_geocentricLatTextBox.ReadOnly = true;
            this.m_geocentricLatTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_geocentricLatTextBox.TabIndex = 33;
            //
            // m_rectifyingLatTextBox
            //
            this.m_rectifyingLatTextBox.Location = new System.Drawing.Point(512, 82);
            this.m_rectifyingLatTextBox.Name = "m_rectifyingLatTextBox";
            this.m_rectifyingLatTextBox.ReadOnly = true;
            this.m_rectifyingLatTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_rectifyingLatTextBox.TabIndex = 34;
            //
            // m_authalicLatTextBox
            //
            this.m_authalicLatTextBox.Location = new System.Drawing.Point(512, 108);
            this.m_authalicLatTextBox.Name = "m_authalicLatTextBox";
            this.m_authalicLatTextBox.ReadOnly = true;
            this.m_authalicLatTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_authalicLatTextBox.TabIndex = 35;
            //
            // m_conformalTextBox
            //
            this.m_conformalTextBox.Location = new System.Drawing.Point(512, 134);
            this.m_conformalTextBox.Name = "m_conformalTextBox";
            this.m_conformalTextBox.ReadOnly = true;
            this.m_conformalTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_conformalTextBox.TabIndex = 36;
            //
            // m_isometricLatTextBox
            //
            this.m_isometricLatTextBox.Location = new System.Drawing.Point(512, 160);
            this.m_isometricLatTextBox.Name = "m_isometricLatTextBox";
            this.m_isometricLatTextBox.ReadOnly = true;
            this.m_isometricLatTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_isometricLatTextBox.TabIndex = 37;
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(435, 185);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(126, 23);
            this.button1.TabIndex = 38;
            this.button1.Text = "Calculate Latitudes";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnCalculateLatitudes);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(18, 150);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 39;
            this.button2.Text = "Validate";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnValidate);
            //
            // EllipsoidPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_isometricLatTextBox);
            this.Controls.Add(this.m_conformalTextBox);
            this.Controls.Add(this.m_authalicLatTextBox);
            this.Controls.Add(this.m_rectifyingLatTextBox);
            this.Controls.Add(this.m_geocentricLatTextBox);
            this.Controls.Add(this.m_parametericLatTextBox);
            this.Controls.Add(this.label17);
            this.Controls.Add(this.label16);
            this.Controls.Add(this.label15);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.m_phiTextBox);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.m_2ecc2TextBox);
            this.Controls.Add(this.m_ecc2TextBox);
            this.Controls.Add(this.m_3rdFlatTextBox);
            this.Controls.Add(this.m_2ndFlatTextBox);
            this.Controls.Add(this.m_volumeTextBox);
            this.Controls.Add(this.m_areaTextBox);
            this.Controls.Add(this.m_quarterMeridianTextBox);
            this.Controls.Add(this.m_minorRadiusTextBox);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.groupBox1);
            this.Name = "EllipsoidPanel";
            this.Size = new System.Drawing.Size(798, 356);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ToolTip m_toolTip;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button m_setButton;
        private System.Windows.Forms.TextBox m_flatteningTextBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox m_majorRadiusTextBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox m_minorRadiusTextBox;
        private System.Windows.Forms.TextBox m_quarterMeridianTextBox;
        private System.Windows.Forms.TextBox m_areaTextBox;
        private System.Windows.Forms.TextBox m_volumeTextBox;
        private System.Windows.Forms.TextBox m_2ndFlatTextBox;
        private System.Windows.Forms.TextBox m_3rdFlatTextBox;
        private System.Windows.Forms.TextBox m_ecc2TextBox;
        private System.Windows.Forms.TextBox m_2ecc2TextBox;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox m_phiTextBox;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.Label label16;
        private System.Windows.Forms.Label label17;
        private System.Windows.Forms.TextBox m_parametericLatTextBox;
        private System.Windows.Forms.TextBox m_geocentricLatTextBox;
        private System.Windows.Forms.TextBox m_rectifyingLatTextBox;
        private System.Windows.Forms.TextBox m_authalicLatTextBox;
        private System.Windows.Forms.TextBox m_conformalTextBox;
        private System.Windows.Forms.TextBox m_isometricLatTextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
    }
}
