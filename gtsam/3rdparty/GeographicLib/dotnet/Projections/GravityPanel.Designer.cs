namespace Projections
{
    partial class GravityPanel
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
            this.button1 = new System.Windows.Forms.Button();
            this.m_updateButton = new System.Windows.Forms.Button();
            this.m_gravityModelNameTextBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.m_nameTextBox = new System.Windows.Forms.TextBox();
            this.m_descriptionTextBox = new System.Windows.Forms.TextBox();
            this.m_dateTextBox = new System.Windows.Forms.TextBox();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.m_longitudetextBoxT = new System.Windows.Forms.TextBox();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_altitudeTextBox = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.label11 = new System.Windows.Forms.Label();
            this.m_accelXTextBox = new System.Windows.Forms.TextBox();
            this.m_accelYTextBox = new System.Windows.Forms.TextBox();
            this.m_accelZTextBox = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.m_geoidTextBox = new System.Windows.Forms.TextBox();
            this.m_normGravButton = new System.Windows.Forms.Button();
            this.m_GravityCircleButton = new System.Windows.Forms.Button();
            this.m_validateButton = new System.Windows.Forms.Button();
            this.SuspendLayout();
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(391, 21);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(29, 23);
            this.button1.TabIndex = 2;
            this.button1.Text = "...";
            this.toolTip1.SetToolTip(this.button1, "Select gravity model file");
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnSelectGravityModel);
            //
            // m_updateButton
            //
            this.m_updateButton.Enabled = false;
            this.m_updateButton.Location = new System.Drawing.Point(565, 106);
            this.m_updateButton.Name = "m_updateButton";
            this.m_updateButton.Size = new System.Drawing.Size(75, 23);
            this.m_updateButton.TabIndex = 24;
            this.m_updateButton.Text = "Update";
            this.toolTip1.SetToolTip(this.m_updateButton, "Calculate acceleration and geoid height");
            this.m_updateButton.UseVisualStyleBackColor = true;
            this.m_updateButton.Click += new System.EventHandler(this.OnUpdate);
            //
            // m_gravityModelNameTextBox
            //
            this.m_gravityModelNameTextBox.Location = new System.Drawing.Point(7, 22);
            this.m_gravityModelNameTextBox.Name = "m_gravityModelNameTextBox";
            this.m_gravityModelNameTextBox.Size = new System.Drawing.Size(377, 20);
            this.m_gravityModelNameTextBox.TabIndex = 0;
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(4, 6);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(72, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Gravity Model";
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(7, 52);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(35, 13);
            this.label2.TabIndex = 3;
            this.label2.Text = "Name";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(10, 79);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(60, 13);
            this.label3.TabIndex = 4;
            this.label3.Text = "Description";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(13, 106);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(30, 13);
            this.label4.TabIndex = 5;
            this.label4.Text = "Date";
            //
            // m_nameTextBox
            //
            this.m_nameTextBox.Location = new System.Drawing.Point(85, 48);
            this.m_nameTextBox.Name = "m_nameTextBox";
            this.m_nameTextBox.ReadOnly = true;
            this.m_nameTextBox.Size = new System.Drawing.Size(299, 20);
            this.m_nameTextBox.TabIndex = 6;
            //
            // m_descriptionTextBox
            //
            this.m_descriptionTextBox.Location = new System.Drawing.Point(85, 75);
            this.m_descriptionTextBox.Name = "m_descriptionTextBox";
            this.m_descriptionTextBox.ReadOnly = true;
            this.m_descriptionTextBox.Size = new System.Drawing.Size(299, 20);
            this.m_descriptionTextBox.TabIndex = 7;
            //
            // m_dateTextBox
            //
            this.m_dateTextBox.Location = new System.Drawing.Point(85, 102);
            this.m_dateTextBox.Name = "m_dateTextBox";
            this.m_dateTextBox.ReadOnly = true;
            this.m_dateTextBox.Size = new System.Drawing.Size(299, 20);
            this.m_dateTextBox.TabIndex = 8;
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(433, 32);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(101, 13);
            this.label5.TabIndex = 9;
            this.label5.Text = "Longitude (degrees)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(433, 58);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(92, 13);
            this.label6.TabIndex = 10;
            this.label6.Text = "Latitude (degrees)";
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(433, 84);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(82, 13);
            this.label7.TabIndex = 11;
            this.label7.Text = "Altitude (meters)";
            //
            // m_longitudetextBoxT
            //
            this.m_longitudetextBoxT.Location = new System.Drawing.Point(539, 28);
            this.m_longitudetextBoxT.Name = "m_longitudetextBoxT";
            this.m_longitudetextBoxT.Size = new System.Drawing.Size(126, 20);
            this.m_longitudetextBoxT.TabIndex = 12;
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(539, 54);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(126, 20);
            this.m_latitudeTextBox.TabIndex = 13;
            //
            // m_altitudeTextBox
            //
            this.m_altitudeTextBox.Location = new System.Drawing.Point(539, 80);
            this.m_altitudeTextBox.Name = "m_altitudeTextBox";
            this.m_altitudeTextBox.Size = new System.Drawing.Size(126, 20);
            this.m_altitudeTextBox.TabIndex = 14;
            //
            // label8
            //
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(670, 32);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(14, 13);
            this.label8.TabIndex = 15;
            this.label8.Text = "X";
            //
            // label9
            //
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(670, 58);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(14, 13);
            this.label9.TabIndex = 16;
            this.label9.Text = "Y";
            //
            // label10
            //
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(670, 84);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(14, 13);
            this.label10.TabIndex = 17;
            this.label10.Text = "Z";
            //
            // label11
            //
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(700, 6);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(105, 13);
            this.label11.TabIndex = 18;
            this.label11.Text = "Acceleration (m/s^2)";
            //
            // m_accelXTextBox
            //
            this.m_accelXTextBox.Location = new System.Drawing.Point(691, 28);
            this.m_accelXTextBox.Name = "m_accelXTextBox";
            this.m_accelXTextBox.ReadOnly = true;
            this.m_accelXTextBox.Size = new System.Drawing.Size(122, 20);
            this.m_accelXTextBox.TabIndex = 19;
            //
            // m_accelYTextBox
            //
            this.m_accelYTextBox.Location = new System.Drawing.Point(690, 54);
            this.m_accelYTextBox.Name = "m_accelYTextBox";
            this.m_accelYTextBox.ReadOnly = true;
            this.m_accelYTextBox.Size = new System.Drawing.Size(122, 20);
            this.m_accelYTextBox.TabIndex = 20;
            //
            // m_accelZTextBox
            //
            this.m_accelZTextBox.Location = new System.Drawing.Point(691, 80);
            this.m_accelZTextBox.Name = "m_accelZTextBox";
            this.m_accelZTextBox.ReadOnly = true;
            this.m_accelZTextBox.Size = new System.Drawing.Size(122, 20);
            this.m_accelZTextBox.TabIndex = 21;
            //
            // label12
            //
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(709, 106);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(86, 13);
            this.label12.TabIndex = 22;
            this.label12.Text = "Geoid Height (m)";
            //
            // m_geoidTextBox
            //
            this.m_geoidTextBox.Location = new System.Drawing.Point(691, 125);
            this.m_geoidTextBox.Name = "m_geoidTextBox";
            this.m_geoidTextBox.ReadOnly = true;
            this.m_geoidTextBox.Size = new System.Drawing.Size(122, 20);
            this.m_geoidTextBox.TabIndex = 23;
            //
            // m_normGravButton
            //
            this.m_normGravButton.Enabled = false;
            this.m_normGravButton.Location = new System.Drawing.Point(544, 135);
            this.m_normGravButton.Name = "m_normGravButton";
            this.m_normGravButton.Size = new System.Drawing.Size(117, 42);
            this.m_normGravButton.TabIndex = 25;
            this.m_normGravButton.Text = "Acceleration using NormalGravity";
            this.m_normGravButton.UseVisualStyleBackColor = true;
            this.m_normGravButton.Click += new System.EventHandler(this.OnNormGravity);
            //
            // m_GravityCircleButton
            //
            this.m_GravityCircleButton.Enabled = false;
            this.m_GravityCircleButton.Location = new System.Drawing.Point(403, 135);
            this.m_GravityCircleButton.Name = "m_GravityCircleButton";
            this.m_GravityCircleButton.Size = new System.Drawing.Size(131, 54);
            this.m_GravityCircleButton.TabIndex = 26;
            this.m_GravityCircleButton.Text = "Acceleration and geoid height using GravityCircle";
            this.m_GravityCircleButton.UseVisualStyleBackColor = true;
            this.m_GravityCircleButton.Click += new System.EventHandler(this.OnGravityCircle);
            //
            // m_validateButton
            //
            this.m_validateButton.Enabled = false;
            this.m_validateButton.Location = new System.Drawing.Point(43, 145);
            this.m_validateButton.Name = "m_validateButton";
            this.m_validateButton.Size = new System.Drawing.Size(75, 23);
            this.m_validateButton.TabIndex = 27;
            this.m_validateButton.Text = "Validate";
            this.m_validateButton.UseVisualStyleBackColor = true;
            this.m_validateButton.Click += new System.EventHandler(this.OnValidate);
            //
            // GravityPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_validateButton);
            this.Controls.Add(this.m_GravityCircleButton);
            this.Controls.Add(this.m_normGravButton);
            this.Controls.Add(this.m_updateButton);
            this.Controls.Add(this.m_geoidTextBox);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.m_accelZTextBox);
            this.Controls.Add(this.m_accelYTextBox);
            this.Controls.Add(this.m_accelXTextBox);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.m_altitudeTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.m_longitudetextBoxT);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.m_dateTextBox);
            this.Controls.Add(this.m_descriptionTextBox);
            this.Controls.Add(this.m_nameTextBox);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.m_gravityModelNameTextBox);
            this.Name = "GravityPanel";
            this.Size = new System.Drawing.Size(1083, 403);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.TextBox m_gravityModelNameTextBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox m_nameTextBox;
        private System.Windows.Forms.TextBox m_descriptionTextBox;
        private System.Windows.Forms.TextBox m_dateTextBox;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox m_longitudetextBoxT;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_altitudeTextBox;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox m_accelXTextBox;
        private System.Windows.Forms.TextBox m_accelYTextBox;
        private System.Windows.Forms.TextBox m_accelZTextBox;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox m_geoidTextBox;
        private System.Windows.Forms.Button m_updateButton;
        private System.Windows.Forms.Button m_normGravButton;
        private System.Windows.Forms.Button m_GravityCircleButton;
        private System.Windows.Forms.Button m_validateButton;
    }
}
