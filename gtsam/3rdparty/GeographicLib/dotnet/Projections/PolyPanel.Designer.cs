namespace Projections
{
    partial class PolyPanel
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
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.m_setButton = new System.Windows.Forms.Button();
            this.m_flatteningTextBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.m_majorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.m_azimuthTextBox = new System.Windows.Forms.TextBox();
            this.m_distanceTextBox = new System.Windows.Forms.TextBox();
            this.m_edgeButton = new System.Windows.Forms.Button();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.m_numPointsTextBox = new System.Windows.Forms.TextBox();
            this.m_currLatTextBox = new System.Windows.Forms.TextBox();
            this.m_currLonTextBox = new System.Windows.Forms.TextBox();
            this.m_lengthTextBox = new System.Windows.Forms.TextBox();
            this.m_areaTextBox = new System.Windows.Forms.TextBox();
            this.button2 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.m_polylineCheckBox = new System.Windows.Forms.CheckBox();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            //
            // groupBox1
            //
            this.groupBox1.Controls.Add(this.m_polylineCheckBox);
            this.groupBox1.Controls.Add(this.m_setButton);
            this.groupBox1.Controls.Add(this.m_flatteningTextBox);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.m_majorRadiusTextBox);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Location = new System.Drawing.Point(3, 3);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(146, 169);
            this.groupBox1.TabIndex = 6;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Ellipsoid Parameters";
            //
            // m_setButton
            //
            this.m_setButton.Location = new System.Drawing.Point(30, 135);
            this.m_setButton.Name = "m_setButton";
            this.m_setButton.Size = new System.Drawing.Size(75, 23);
            this.m_setButton.TabIndex = 4;
            this.m_setButton.Text = "Set";
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
            this.label3.Location = new System.Drawing.Point(154, 10);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(92, 13);
            this.label3.TabIndex = 7;
            this.label3.Text = "Latitude (degrees)";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(154, 36);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(101, 13);
            this.label4.TabIndex = 8;
            this.label4.Text = "Longitude (degrees)";
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(263, 6);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_latitudeTextBox.TabIndex = 9;
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(263, 32);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_longitudeTextBox.TabIndex = 10;
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(289, 60);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 11;
            this.button1.Text = "Add Point";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnAddPoint);
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(400, 8);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(91, 13);
            this.label5.TabIndex = 12;
            this.label5.Text = "Azimuth (degrees)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(400, 34);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(89, 13);
            this.label6.TabIndex = 13;
            this.label6.Text = "Distance (meters)";
            //
            // m_azimuthTextBox
            //
            this.m_azimuthTextBox.Location = new System.Drawing.Point(495, 4);
            this.m_azimuthTextBox.Name = "m_azimuthTextBox";
            this.m_azimuthTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_azimuthTextBox.TabIndex = 14;
            //
            // m_distanceTextBox
            //
            this.m_distanceTextBox.Location = new System.Drawing.Point(495, 30);
            this.m_distanceTextBox.Name = "m_distanceTextBox";
            this.m_distanceTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_distanceTextBox.TabIndex = 15;
            //
            // m_edgeButton
            //
            this.m_edgeButton.Enabled = false;
            this.m_edgeButton.Location = new System.Drawing.Point(515, 58);
            this.m_edgeButton.Name = "m_edgeButton";
            this.m_edgeButton.Size = new System.Drawing.Size(75, 23);
            this.m_edgeButton.TabIndex = 16;
            this.m_edgeButton.Text = "Add Edge";
            this.m_edgeButton.UseVisualStyleBackColor = true;
            this.m_edgeButton.Click += new System.EventHandler(this.OnAddEdge);
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(634, 8);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(87, 13);
            this.label7.TabIndex = 17;
            this.label7.Text = "Number of points";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(634, 34);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(82, 13);
            this.label8.TabIndex = 18;
            this.label8.Text = "Current Latitude";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(634, 61);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(91, 13);
            this.label9.TabIndex = 19;
            this.label9.Text = "Current Longitude";
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(634, 89);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(40, 13);
            this.label10.TabIndex = 20;
            this.label10.Text = "Length";
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(634, 116);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(29, 13);
            this.label11.TabIndex = 21;
            this.label11.Text = "Area";
            //
            // m_numPointsTextBox
            //
            this.m_numPointsTextBox.Location = new System.Drawing.Point(729, 4);
            this.m_numPointsTextBox.Name = "m_numPointsTextBox";
            this.m_numPointsTextBox.ReadOnly = true;
            this.m_numPointsTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_numPointsTextBox.TabIndex = 22;
            this.m_numPointsTextBox.Text = "0";
            //
            // m_currLatTextBox
            //
            this.m_currLatTextBox.Location = new System.Drawing.Point(729, 30);
            this.m_currLatTextBox.Name = "m_currLatTextBox";
            this.m_currLatTextBox.ReadOnly = true;
            this.m_currLatTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_currLatTextBox.TabIndex = 23;
            //
            // m_currLonTextBox
            //
            this.m_currLonTextBox.Location = new System.Drawing.Point(729, 57);
            this.m_currLonTextBox.Name = "m_currLonTextBox";
            this.m_currLonTextBox.ReadOnly = true;
            this.m_currLonTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_currLonTextBox.TabIndex = 24;
            //
            // m_lengthTextBox
            //
            this.m_lengthTextBox.Location = new System.Drawing.Point(729, 85);
            this.m_lengthTextBox.Name = "m_lengthTextBox";
            this.m_lengthTextBox.ReadOnly = true;
            this.m_lengthTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_lengthTextBox.TabIndex = 25;
            //
            // m_areaTextBox
            //
            this.m_areaTextBox.Location = new System.Drawing.Point(729, 112);
            this.m_areaTextBox.Name = "m_areaTextBox";
            this.m_areaTextBox.ReadOnly = true;
            this.m_areaTextBox.Size = new System.Drawing.Size(127, 20);
            this.m_areaTextBox.TabIndex = 26;
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(405, 84);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 27;
            this.button2.Text = "Clear";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnClear);
            //
            // button3
            //
            this.button3.Location = new System.Drawing.Point(405, 109);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 28;
            this.button3.Text = "Validate";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.OnValidate);
            //
            // m_polylineCheckBox
            //
            this.m_polylineCheckBox.AutoSize = true;
            this.m_polylineCheckBox.Location = new System.Drawing.Point(15, 112);
            this.m_polylineCheckBox.Name = "m_polylineCheckBox";
            this.m_polylineCheckBox.Size = new System.Drawing.Size(62, 17);
            this.m_polylineCheckBox.TabIndex = 29;
            this.m_polylineCheckBox.Text = "Polyline";
            this.m_polylineCheckBox.UseVisualStyleBackColor = true;
            //
            // PolyPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.m_areaTextBox);
            this.Controls.Add(this.m_lengthTextBox);
            this.Controls.Add(this.m_currLonTextBox);
            this.Controls.Add(this.m_currLatTextBox);
            this.Controls.Add(this.m_numPointsTextBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.m_edgeButton);
            this.Controls.Add(this.m_distanceTextBox);
            this.Controls.Add(this.m_azimuthTextBox);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.groupBox1);
            this.Name = "PolyPanel";
            this.Size = new System.Drawing.Size(979, 359);
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
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox m_azimuthTextBox;
        private System.Windows.Forms.TextBox m_distanceTextBox;
        private System.Windows.Forms.Button m_edgeButton;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox m_numPointsTextBox;
        private System.Windows.Forms.TextBox m_currLatTextBox;
        private System.Windows.Forms.TextBox m_currLonTextBox;
        private System.Windows.Forms.TextBox m_lengthTextBox;
        private System.Windows.Forms.TextBox m_areaTextBox;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.CheckBox m_polylineCheckBox;
    }
}
