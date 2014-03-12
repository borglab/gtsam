namespace Projections
{
    partial class MagneticPanel
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
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.m_validateButton = new System.Windows.Forms.Button();
            this.m_dateTextBox = new System.Windows.Forms.TextBox();
            this.m_descriptionTextBox = new System.Windows.Forms.TextBox();
            this.m_nameTextBox = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.button1 = new System.Windows.Forms.Button();
            this.label1 = new System.Windows.Forms.Label();
            this.m_magneticModelNameTextBox = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.m_timeTextBox = new System.Windows.Forms.TextBox();
            this.m_longitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_altitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_updateButton = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.m_mfXTextBox = new System.Windows.Forms.TextBox();
            this.m_mfYTextBox = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.m_mfZTextBox = new System.Windows.Forms.TextBox();
            this.m_mfdXTextBox = new System.Windows.Forms.TextBox();
            this.m_mfdYTextBox = new System.Windows.Forms.TextBox();
            this.m_mfdZTextBox = new System.Windows.Forms.TextBox();
            this.m_magCircButton = new System.Windows.Forms.Button();
            this.SuspendLayout();
            //
            // m_validateButton
            //
            this.m_validateButton.Enabled = false;
            this.m_validateButton.Location = new System.Drawing.Point(182, 126);
            this.m_validateButton.Name = "m_validateButton";
            this.m_validateButton.Size = new System.Drawing.Size(75, 23);
            this.m_validateButton.TabIndex = 37;
            this.m_validateButton.Text = "Validate";
            this.m_validateButton.UseVisualStyleBackColor = true;
            this.m_validateButton.Click += new System.EventHandler(this.OnValidate);
            //
            // m_dateTextBox
            //
            this.m_dateTextBox.Location = new System.Drawing.Point(82, 100);
            this.m_dateTextBox.Name = "m_dateTextBox";
            this.m_dateTextBox.ReadOnly = true;
            this.m_dateTextBox.Size = new System.Drawing.Size(299, 20);
            this.m_dateTextBox.TabIndex = 36;
            //
            // m_descriptionTextBox
            //
            this.m_descriptionTextBox.Location = new System.Drawing.Point(82, 73);
            this.m_descriptionTextBox.Name = "m_descriptionTextBox";
            this.m_descriptionTextBox.ReadOnly = true;
            this.m_descriptionTextBox.Size = new System.Drawing.Size(299, 20);
            this.m_descriptionTextBox.TabIndex = 35;
            //
            // m_nameTextBox
            //
            this.m_nameTextBox.Location = new System.Drawing.Point(82, 46);
            this.m_nameTextBox.Name = "m_nameTextBox";
            this.m_nameTextBox.ReadOnly = true;
            this.m_nameTextBox.Size = new System.Drawing.Size(299, 20);
            this.m_nameTextBox.TabIndex = 34;
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(9, 104);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(30, 13);
            this.label4.TabIndex = 33;
            this.label4.Text = "Date";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(9, 77);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(60, 13);
            this.label3.TabIndex = 32;
            this.label3.Text = "Description";
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(9, 50);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(35, 13);
            this.label2.TabIndex = 31;
            this.label2.Text = "Name";
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(388, 19);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(29, 23);
            this.button1.TabIndex = 30;
            this.button1.Text = "...";
            this.toolTip1.SetToolTip(this.button1, "Select gravity model file");
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnSelectMagneticModel);
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(9, 4);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(83, 13);
            this.label1.TabIndex = 29;
            this.label1.Text = "Magnetic Model";
            //
            // m_magneticModelNameTextBox
            //
            this.m_magneticModelNameTextBox.Location = new System.Drawing.Point(9, 20);
            this.m_magneticModelNameTextBox.Name = "m_magneticModelNameTextBox";
            this.m_magneticModelNameTextBox.Size = new System.Drawing.Size(377, 20);
            this.m_magneticModelNameTextBox.TabIndex = 28;
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(429, 8);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(64, 13);
            this.label5.TabIndex = 38;
            this.label5.Text = "Time (years)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(429, 34);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(101, 13);
            this.label6.TabIndex = 39;
            this.label6.Text = "Longitude (degrees)";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(429, 60);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(92, 13);
            this.label7.TabIndex = 40;
            this.label7.Text = "Latitude (degrees)";
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(429, 86);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(82, 13);
            this.label8.TabIndex = 41;
            this.label8.Text = "Altitude (meters)";
            //
            // m_timeTextBox
            //
            this.m_timeTextBox.Location = new System.Drawing.Point(544, 4);
            this.m_timeTextBox.Name = "m_timeTextBox";
            this.m_timeTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_timeTextBox.TabIndex = 42;
            //
            // m_longitudeTextBox
            //
            this.m_longitudeTextBox.Location = new System.Drawing.Point(544, 30);
            this.m_longitudeTextBox.Name = "m_longitudeTextBox";
            this.m_longitudeTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_longitudeTextBox.TabIndex = 43;
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(544, 56);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_latitudeTextBox.TabIndex = 44;
            //
            // m_altitudeTextBox
            //
            this.m_altitudeTextBox.Location = new System.Drawing.Point(544, 82);
            this.m_altitudeTextBox.Name = "m_altitudeTextBox";
            this.m_altitudeTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_altitudeTextBox.TabIndex = 45;
            //
            // m_updateButton
            //
            this.m_updateButton.Enabled = false;
            this.m_updateButton.Location = new System.Drawing.Point(566, 108);
            this.m_updateButton.Name = "m_updateButton";
            this.m_updateButton.Size = new System.Drawing.Size(75, 23);
            this.m_updateButton.TabIndex = 46;
            this.m_updateButton.Text = "Update";
            this.m_updateButton.UseVisualStyleBackColor = true;
            this.m_updateButton.Click += new System.EventHandler(this.OnUpdate);
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(709, 8);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(76, 13);
            this.label9.TabIndex = 47;
            this.label9.Text = "Magnetic Field";
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(814, 8);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(116, 13);
            this.label10.TabIndex = 48;
            this.label10.Text = "Magnetic Field Change";
            //
            // m_mfXTextBox
            //
            this.m_mfXTextBox.Location = new System.Drawing.Point(688, 30);
            this.m_mfXTextBox.Name = "m_mfXTextBox";
            this.m_mfXTextBox.ReadOnly = true;
            this.m_mfXTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_mfXTextBox.TabIndex = 49;
            //
            // m_mfYTextBox
            //
            this.m_mfYTextBox.Location = new System.Drawing.Point(688, 56);
            this.m_mfYTextBox.Name = "m_mfYTextBox";
            this.m_mfYTextBox.ReadOnly = true;
            this.m_mfYTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_mfYTextBox.TabIndex = 50;
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(668, 34);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(14, 13);
            this.label11.TabIndex = 51;
            this.label11.Text = "X";
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(668, 60);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(14, 13);
            this.label12.TabIndex = 52;
            this.label12.Text = "Y";
            //
            // label13
            //
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(668, 86);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(14, 13);
            this.label13.TabIndex = 53;
            this.label13.Text = "Z";
            //
            // m_mfZTextBox
            //
            this.m_mfZTextBox.Location = new System.Drawing.Point(688, 82);
            this.m_mfZTextBox.Name = "m_mfZTextBox";
            this.m_mfZTextBox.ReadOnly = true;
            this.m_mfZTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_mfZTextBox.TabIndex = 54;
            //
            // m_mfdXTextBox
            //
            this.m_mfdXTextBox.Location = new System.Drawing.Point(813, 30);
            this.m_mfdXTextBox.Name = "m_mfdXTextBox";
            this.m_mfdXTextBox.ReadOnly = true;
            this.m_mfdXTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_mfdXTextBox.TabIndex = 55;
            //
            // m_mfdYTextBox
            //
            this.m_mfdYTextBox.Location = new System.Drawing.Point(813, 56);
            this.m_mfdYTextBox.Name = "m_mfdYTextBox";
            this.m_mfdYTextBox.ReadOnly = true;
            this.m_mfdYTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_mfdYTextBox.TabIndex = 56;
            //
            // m_mfdZTextBox
            //
            this.m_mfdZTextBox.Location = new System.Drawing.Point(813, 82);
            this.m_mfdZTextBox.Name = "m_mfdZTextBox";
            this.m_mfdZTextBox.ReadOnly = true;
            this.m_mfdZTextBox.Size = new System.Drawing.Size(118, 20);
            this.m_mfdZTextBox.TabIndex = 57;
            //
            // m_magCircButton
            //
            this.m_magCircButton.Enabled = false;
            this.m_magCircButton.Location = new System.Drawing.Point(712, 108);
            this.m_magCircButton.Name = "m_magCircButton";
            this.m_magCircButton.Size = new System.Drawing.Size(197, 23);
            this.m_magCircButton.TabIndex = 58;
            this.m_magCircButton.Text = "Use MagneticCircle to update Field";
            this.m_magCircButton.UseVisualStyleBackColor = true;
            this.m_magCircButton.Click += new System.EventHandler(this.OnMagneticCircle);
            //
            // MagneticPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_magCircButton);
            this.Controls.Add(this.m_mfdZTextBox);
            this.Controls.Add(this.m_mfdYTextBox);
            this.Controls.Add(this.m_mfdXTextBox);
            this.Controls.Add(this.m_mfZTextBox);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.m_mfYTextBox);
            this.Controls.Add(this.m_mfXTextBox);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.m_updateButton);
            this.Controls.Add(this.m_altitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.m_longitudeTextBox);
            this.Controls.Add(this.m_timeTextBox);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.m_validateButton);
            this.Controls.Add(this.m_dateTextBox);
            this.Controls.Add(this.m_descriptionTextBox);
            this.Controls.Add(this.m_nameTextBox);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.m_magneticModelNameTextBox);
            this.Name = "MagneticPanel";
            this.Size = new System.Drawing.Size(984, 428);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.Button m_validateButton;
        private System.Windows.Forms.TextBox m_dateTextBox;
        private System.Windows.Forms.TextBox m_descriptionTextBox;
        private System.Windows.Forms.TextBox m_nameTextBox;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox m_magneticModelNameTextBox;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.TextBox m_timeTextBox;
        private System.Windows.Forms.TextBox m_longitudeTextBox;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_altitudeTextBox;
        private System.Windows.Forms.Button m_updateButton;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox m_mfXTextBox;
        private System.Windows.Forms.TextBox m_mfYTextBox;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox m_mfZTextBox;
        private System.Windows.Forms.TextBox m_mfdXTextBox;
        private System.Windows.Forms.TextBox m_mfdYTextBox;
        private System.Windows.Forms.TextBox m_mfdZTextBox;
        private System.Windows.Forms.Button m_magCircButton;
    }
}
