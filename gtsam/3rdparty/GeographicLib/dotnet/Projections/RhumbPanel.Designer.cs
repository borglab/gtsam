namespace Projections
{
    partial class RhumbPanel
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
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.m_Lat1TextBox = new System.Windows.Forms.TextBox();
            this.m_lon1TextBox = new System.Windows.Forms.TextBox();
            this.m_lat2TextBox = new System.Windows.Forms.TextBox();
            this.m_lon2TextBox = new System.Windows.Forms.TextBox();
            this.m_s12TextBox = new System.Windows.Forms.TextBox();
            this.m_azimuthTextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.label7 = new System.Windows.Forms.Label();
            this.m_pointsView = new System.Windows.Forms.ListView();
            this.columnHeader1 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.columnHeader2 = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.button3 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(16, 15);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(101, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Latitude 1 (degrees)";
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(16, 42);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(110, 13);
            this.label2.TabIndex = 1;
            this.label2.Text = "Longitude 1 (degrees)";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(16, 69);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(101, 13);
            this.label3.TabIndex = 2;
            this.label3.Text = "Latitude 2 (degrees)";
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(16, 96);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(110, 13);
            this.label4.TabIndex = 3;
            this.label4.Text = "Longitude 2 (degrees)";
            //
            // label5
            //
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(16, 123);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(66, 13);
            this.label5.TabIndex = 4;
            this.label5.Text = "S12 (meters)";
            //
            // label6
            //
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(16, 150);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(91, 13);
            this.label6.TabIndex = 5;
            this.label6.Text = "Azimuth (degrees)";
            //
            // m_Lat1TextBox
            //
            this.m_Lat1TextBox.Location = new System.Drawing.Point(131, 11);
            this.m_Lat1TextBox.Name = "m_Lat1TextBox";
            this.m_Lat1TextBox.Size = new System.Drawing.Size(129, 20);
            this.m_Lat1TextBox.TabIndex = 6;
            //
            // m_lon1TextBox
            //
            this.m_lon1TextBox.Location = new System.Drawing.Point(131, 38);
            this.m_lon1TextBox.Name = "m_lon1TextBox";
            this.m_lon1TextBox.Size = new System.Drawing.Size(129, 20);
            this.m_lon1TextBox.TabIndex = 7;
            //
            // m_lat2TextBox
            //
            this.m_lat2TextBox.Location = new System.Drawing.Point(131, 65);
            this.m_lat2TextBox.Name = "m_lat2TextBox";
            this.m_lat2TextBox.Size = new System.Drawing.Size(129, 20);
            this.m_lat2TextBox.TabIndex = 8;
            //
            // m_lon2TextBox
            //
            this.m_lon2TextBox.Location = new System.Drawing.Point(131, 92);
            this.m_lon2TextBox.Name = "m_lon2TextBox";
            this.m_lon2TextBox.Size = new System.Drawing.Size(129, 20);
            this.m_lon2TextBox.TabIndex = 9;
            //
            // m_s12TextBox
            //
            this.m_s12TextBox.Location = new System.Drawing.Point(131, 119);
            this.m_s12TextBox.Name = "m_s12TextBox";
            this.m_s12TextBox.Size = new System.Drawing.Size(129, 20);
            this.m_s12TextBox.TabIndex = 10;
            //
            // m_azimuthTextBox
            //
            this.m_azimuthTextBox.Location = new System.Drawing.Point(131, 146);
            this.m_azimuthTextBox.Name = "m_azimuthTextBox";
            this.m_azimuthTextBox.Size = new System.Drawing.Size(129, 20);
            this.m_azimuthTextBox.TabIndex = 11;
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(19, 178);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 12;
            this.button1.Text = "Direct";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnDirect);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(131, 177);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 13;
            this.button2.Text = "Inverse";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnIndirect);
            //
            // label7
            //
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(306, 17);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(96, 13);
            this.label7.TabIndex = 14;
            this.label7.Text = "Rhumb Line Points";
            //
            // m_pointsView
            //
            this.m_pointsView.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.columnHeader1,
            this.columnHeader2});
            this.m_pointsView.Location = new System.Drawing.Point(309, 38);
            this.m_pointsView.Name = "m_pointsView";
            this.m_pointsView.Size = new System.Drawing.Size(321, 128);
            this.m_pointsView.TabIndex = 15;
            this.m_pointsView.UseCompatibleStateImageBehavior = false;
            this.m_pointsView.View = System.Windows.Forms.View.Details;
            //
            // columnHeader1
            //
            this.columnHeader1.Text = "Latitude (deg)";
            this.columnHeader1.Width = 161;
            //
            // columnHeader2
            //
            this.columnHeader2.Text = "Longitude (deg)";
            this.columnHeader2.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.columnHeader2.Width = 120;
            //
            // button3
            //
            this.button3.Location = new System.Drawing.Point(309, 177);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 16;
            this.button3.Text = "Validate";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.OnValidate);
            //
            // RhumbPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.button3);
            this.Controls.Add(this.m_pointsView);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_azimuthTextBox);
            this.Controls.Add(this.m_s12TextBox);
            this.Controls.Add(this.m_lon2TextBox);
            this.Controls.Add(this.m_lat2TextBox);
            this.Controls.Add(this.m_lon1TextBox);
            this.Controls.Add(this.m_Lat1TextBox);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Name = "RhumbPanel";
            this.Size = new System.Drawing.Size(662, 287);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox m_Lat1TextBox;
        private System.Windows.Forms.TextBox m_lon1TextBox;
        private System.Windows.Forms.TextBox m_lat2TextBox;
        private System.Windows.Forms.TextBox m_lon2TextBox;
        private System.Windows.Forms.TextBox m_s12TextBox;
        private System.Windows.Forms.TextBox m_azimuthTextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.ListView m_pointsView;
        private System.Windows.Forms.ColumnHeader columnHeader1;
        private System.Windows.Forms.ColumnHeader columnHeader2;
        private System.Windows.Forms.Button button3;
    }
}
