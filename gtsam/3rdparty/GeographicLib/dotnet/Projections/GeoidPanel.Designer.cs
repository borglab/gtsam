namespace Projections
{
    partial class GeoidPanel
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
            this.m_geoidFileNameTextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.m_threadSafeCheckBox = new System.Windows.Forms.CheckBox();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.m_dateTimeTextBox = new System.Windows.Forms.TextBox();
            this.m_descriptionTextBox = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.m_majorRadiusTextBox = new System.Windows.Forms.TextBox();
            this.m_flatteningTtextBox = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.m_northTextBox = new System.Windows.Forms.TextBox();
            this.m_southTextBox = new System.Windows.Forms.TextBox();
            this.m_eastTextBox = new System.Windows.Forms.TextBox();
            this.m_westTextBox = new System.Windows.Forms.TextBox();
            this.m_cacheButton = new System.Windows.Forms.Button();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.m_ellipsoidTextBox = new System.Windows.Forms.TextBox();
            this.m_geoidTextBox = new System.Windows.Forms.TextBox();
            this.m_convertEllipsodButton = new System.Windows.Forms.Button();
            this.m_convertGeoidButton = new System.Windows.Forms.Button();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_toolTip = new System.Windows.Forms.ToolTip(this.components);
            this.m_heightButton = new System.Windows.Forms.Button();
            this.m_validateButton = new System.Windows.Forms.Button();
            this.SuspendLayout();
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(6, 5);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(54, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Geoid File";
            //
            // m_geoidFileNameTextBox
            //
            this.m_geoidFileNameTextBox.Location = new System.Drawing.Point(9, 21);
            this.m_geoidFileNameTextBox.Name = "m_geoidFileNameTextBox";
            this.m_geoidFileNameTextBox.Size = new System.Drawing.Size(388, 20);
            this.m_geoidFileNameTextBox.TabIndex = 1;
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(404, 20);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(25, 23);
            this.button1.TabIndex = 2;
            this.button1.Text = "...";
            this.m_toolTip.SetToolTip(this.button1, "Select Geoid File");
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnSelectFile);
            //
            // m_threadSafeCheckBox
            //
            this.m_threadSafeCheckBox.AutoSize = true;
            this.m_threadSafeCheckBox.Location = new System.Drawing.Point(9, 48);
            this.m_threadSafeCheckBox.Name = "m_threadSafeCheckBox";
            this.m_threadSafeCheckBox.Size = new System.Drawing.Size(85, 17);
            this.m_threadSafeCheckBox.TabIndex = 3;
            this.m_threadSafeCheckBox.Text = "Thread Safe";
            this.m_threadSafeCheckBox.UseVisualStyleBackColor = true;
            this.m_threadSafeCheckBox.CheckedChanged += new System.EventHandler(this.OnThreadSafe);
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(7, 72);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(58, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "Date/Time";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(7, 101);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(60, 13);
            this.label3.TabIndex = 5;
            this.label3.Text = "Description";
            //
            // m_dateTimeTextBox
            //
            this.m_dateTimeTextBox.Location = new System.Drawing.Point(80, 68);
            this.m_dateTimeTextBox.Name = "m_dateTimeTextBox";
            this.m_dateTimeTextBox.ReadOnly = true;
            this.m_dateTimeTextBox.Size = new System.Drawing.Size(181, 20);
            this.m_dateTimeTextBox.TabIndex = 6;
            //
            // m_descriptionTextBox
            //
            this.m_descriptionTextBox.Location = new System.Drawing.Point(80, 97);
            this.m_descriptionTextBox.Name = "m_descriptionTextBox";
            this.m_descriptionTextBox.ReadOnly = true;
            this.m_descriptionTextBox.Size = new System.Drawing.Size(349, 20);
            this.m_descriptionTextBox.TabIndex = 7;
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(7, 130);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(69, 13);
            this.label4.TabIndex = 8;
            this.label4.Text = "Major Radius";
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(7, 159);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(53, 13);
            this.label5.TabIndex = 9;
            this.label5.Text = "Flattening";
            //
            // m_majorRadiusTextBox
            //
            this.m_majorRadiusTextBox.Location = new System.Drawing.Point(83, 126);
            this.m_majorRadiusTextBox.Name = "m_majorRadiusTextBox";
            this.m_majorRadiusTextBox.ReadOnly = true;
            this.m_majorRadiusTextBox.Size = new System.Drawing.Size(126, 20);
            this.m_majorRadiusTextBox.TabIndex = 10;
            //
            // m_flatteningTtextBox
            //
            this.m_flatteningTtextBox.Location = new System.Drawing.Point(83, 155);
            this.m_flatteningTtextBox.Name = "m_flatteningTtextBox";
            this.m_flatteningTtextBox.ReadOnly = true;
            this.m_flatteningTtextBox.Size = new System.Drawing.Size(126, 20);
            this.m_flatteningTtextBox.TabIndex = 11;
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(442, 12);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(33, 13);
            this.label6.TabIndex = 12;
            this.label6.Text = "North";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(442, 38);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(35, 13);
            this.label7.TabIndex = 13;
            this.label7.Text = "South";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(442, 64);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(28, 13);
            this.label8.TabIndex = 14;
            this.label8.Text = "East";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(442, 90);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(32, 13);
            this.label9.TabIndex = 15;
            this.label9.Text = "West";
            //
            // m_northTextBox
            //
            this.m_northTextBox.Location = new System.Drawing.Point(482, 8);
            this.m_northTextBox.Name = "m_northTextBox";
            this.m_northTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_northTextBox.TabIndex = 16;
            //
            // m_southTextBox
            //
            this.m_southTextBox.Location = new System.Drawing.Point(482, 34);
            this.m_southTextBox.Name = "m_southTextBox";
            this.m_southTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_southTextBox.TabIndex = 17;
            //
            // m_eastTextBox
            //
            this.m_eastTextBox.Location = new System.Drawing.Point(482, 60);
            this.m_eastTextBox.Name = "m_eastTextBox";
            this.m_eastTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_eastTextBox.TabIndex = 18;
            //
            // m_westTextBox
            //
            this.m_westTextBox.Location = new System.Drawing.Point(482, 86);
            this.m_westTextBox.Name = "m_westTextBox";
            this.m_westTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_westTextBox.TabIndex = 19;
            //
            // m_cacheButton
            //
            this.m_cacheButton.Location = new System.Drawing.Point(492, 113);
            this.m_cacheButton.Name = "m_cacheButton";
            this.m_cacheButton.Size = new System.Drawing.Size(75, 23);
            this.m_cacheButton.TabIndex = 20;
            this.m_cacheButton.Text = "Cache";
            this.m_toolTip.SetToolTip(this.m_cacheButton, "Cache Geoid Data");
            this.m_cacheButton.UseVisualStyleBackColor = true;
            this.m_cacheButton.Click += new System.EventHandler(this.OnCache);
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(606, 63);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(113, 13);
            this.label10.TabIndex = 21;
            this.label10.Text = "Height Above Ellipsoid";
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(754, 63);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(102, 13);
            this.label11.TabIndex = 22;
            this.label11.Text = "Height above Geoid";
            //
            // m_ellipsoidTextBox
            //
            this.m_ellipsoidTextBox.Location = new System.Drawing.Point(596, 85);
            this.m_ellipsoidTextBox.Name = "m_ellipsoidTextBox";
            this.m_ellipsoidTextBox.Size = new System.Drawing.Size(132, 20);
            this.m_ellipsoidTextBox.TabIndex = 23;
            //
            // m_geoidTextBox
            //
            this.m_geoidTextBox.Location = new System.Drawing.Point(739, 85);
            this.m_geoidTextBox.Name = "m_geoidTextBox";
            this.m_geoidTextBox.Size = new System.Drawing.Size(132, 20);
            this.m_geoidTextBox.TabIndex = 24;
            //
            // m_convertEllipsodButton
            //
            this.m_convertEllipsodButton.Enabled = false;
            this.m_convertEllipsodButton.Location = new System.Drawing.Point(625, 109);
            this.m_convertEllipsodButton.Name = "m_convertEllipsodButton";
            this.m_convertEllipsodButton.Size = new System.Drawing.Size(75, 23);
            this.m_convertEllipsodButton.TabIndex = 25;
            this.m_convertEllipsodButton.Text = "Convert ->";
            this.m_toolTip.SetToolTip(this.m_convertEllipsodButton, "Convert Ellipsod Height to Geoid Height");
            this.m_convertEllipsodButton.UseVisualStyleBackColor = true;
            this.m_convertEllipsodButton.Click += new System.EventHandler(this.OnConvertEllipsod);
            //
            // m_convertGeoidButton
            //
            this.m_convertGeoidButton.Enabled = false;
            this.m_convertGeoidButton.Location = new System.Drawing.Point(768, 109);
            this.m_convertGeoidButton.Name = "m_convertGeoidButton";
            this.m_convertGeoidButton.Size = new System.Drawing.Size(75, 23);
            this.m_convertGeoidButton.TabIndex = 26;
            this.m_convertGeoidButton.Text = "<- Convert";
            this.m_toolTip.SetToolTip(this.m_convertGeoidButton, "Convert Geoid Height to Ellipsoid Height");
            this.m_convertGeoidButton.UseVisualStyleBackColor = true;
            this.m_convertGeoidButton.Click += new System.EventHandler(this.OnConvertGeoid);
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(593, 12);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(45, 13);
            this.label12.TabIndex = 27;
            this.label12.Text = "Latitude";
            //
            // label13
            //
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(593, 38);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(54, 13);
            this.label13.TabIndex = 28;
            this.label13.Text = "Longitude";
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(653, 8);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_latitudeTextBox.TabIndex = 29;
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(653, 34);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_longitudeTextBox.TabIndex = 30;
            //
            // m_heightButton
            //
            this.m_heightButton.Enabled = false;
            this.m_heightButton.Location = new System.Drawing.Point(768, 18);
            this.m_heightButton.Name = "m_heightButton";
            this.m_heightButton.Size = new System.Drawing.Size(75, 23);
            this.m_heightButton.TabIndex = 31;
            this.m_heightButton.Text = "Height";
            this.m_toolTip.SetToolTip(this.m_heightButton, "Calculate Geoid Height at Longitude/Latitude");
            this.m_heightButton.UseVisualStyleBackColor = true;
            this.m_heightButton.Click += new System.EventHandler(this.OnHeight);
            //
            // m_validateButton
            //
            this.m_validateButton.Enabled = false;
            this.m_validateButton.Location = new System.Drawing.Point(638, 148);
            this.m_validateButton.Name = "m_validateButton";
            this.m_validateButton.Size = new System.Drawing.Size(75, 23);
            this.m_validateButton.TabIndex = 32;
            this.m_validateButton.Text = "Validate";
            this.m_validateButton.UseVisualStyleBackColor = true;
            this.m_validateButton.Click += new System.EventHandler(this.OnValidate);
            //
            // GeoidPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_validateButton);
            this.Controls.Add(this.m_heightButton);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.m_convertGeoidButton);
            this.Controls.Add(this.m_convertEllipsodButton);
            this.Controls.Add(this.m_geoidTextBox);
            this.Controls.Add(this.m_ellipsoidTextBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.m_cacheButton);
            this.Controls.Add(this.m_westTextBox);
            this.Controls.Add(this.m_eastTextBox);
            this.Controls.Add(this.m_southTextBox);
            this.Controls.Add(this.m_northTextBox);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.m_flatteningTtextBox);
            this.Controls.Add(this.m_majorRadiusTextBox);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.m_descriptionTextBox);
            this.Controls.Add(this.m_dateTimeTextBox);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.m_threadSafeCheckBox);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_geoidFileNameTextBox);
            this.Controls.Add(this.label1);
            this.Name = "GeoidPanel";
            this.Size = new System.Drawing.Size(1002, 273);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox m_geoidFileNameTextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.CheckBox m_threadSafeCheckBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox m_dateTimeTextBox;
        private System.Windows.Forms.TextBox m_descriptionTextBox;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.TextBox m_majorRadiusTextBox;
        private System.Windows.Forms.TextBox m_flatteningTtextBox;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox m_northTextBox;
        private System.Windows.Forms.TextBox m_southTextBox;
        private System.Windows.Forms.TextBox m_eastTextBox;
        private System.Windows.Forms.TextBox m_westTextBox;
        private System.Windows.Forms.Button m_cacheButton;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox m_ellipsoidTextBox;
        private System.Windows.Forms.TextBox m_geoidTextBox;
        private System.Windows.Forms.Button m_convertEllipsodButton;
        private System.Windows.Forms.Button m_convertGeoidButton;
        private System.Windows.Forms.ToolTip m_toolTip;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.Button m_heightButton;
        private System.Windows.Forms.Button m_validateButton;
    }
}
