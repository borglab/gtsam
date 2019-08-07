namespace Projections
{
    partial class MiscPanel
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
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.m_longitudeDMSTextBox = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.m_latitudeDMSTextBox = new System.Windows.Forms.TextBox();
            this.m_LongitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_latitudeTextBox = new System.Windows.Forms.TextBox();
            this.m_geohashTextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.button4 = new System.Windows.Forms.Button();
            this.m_comboBox = new System.Windows.Forms.ComboBox();
            this.SuspendLayout();
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(13, 33);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(54, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Longitude";
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(120, 4);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(31, 13);
            this.label2.TabIndex = 1;
            this.label2.Text = "DMS";
            //
            // label3
            //
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(229, 4);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(88, 13);
            this.label3.TabIndex = 2;
            this.label3.Text = "Decimal Degrees";
            //
            // m_longitudeDMSTextBox
            //
            this.m_longitudeDMSTextBox.Location = new System.Drawing.Point(86, 29);
            this.m_longitudeDMSTextBox.Name = "m_longitudeDMSTextBox";
            this.m_longitudeDMSTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_longitudeDMSTextBox.TabIndex = 3;
            //
            // label4
            //
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(13, 63);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(45, 13);
            this.label4.TabIndex = 4;
            this.label4.Text = "Latitude";
            //
            // m_latitudeDMSTextBox
            //
            this.m_latitudeDMSTextBox.Location = new System.Drawing.Point(86, 56);
            this.m_latitudeDMSTextBox.Name = "m_latitudeDMSTextBox";
            this.m_latitudeDMSTextBox.Size = new System.Drawing.Size(100, 20);
            this.m_latitudeDMSTextBox.TabIndex = 5;
            //
            // m_LongitudeTextBox
            //
            this.m_LongitudeTextBox.Location = new System.Drawing.Point(203, 29);
            this.m_LongitudeTextBox.Name = "m_LongitudeTextBox";
            this.m_LongitudeTextBox.Size = new System.Drawing.Size(137, 20);
            this.m_LongitudeTextBox.TabIndex = 6;
            //
            // m_latitudeTextBox
            //
            this.m_latitudeTextBox.Location = new System.Drawing.Point(203, 55);
            this.m_latitudeTextBox.Name = "m_latitudeTextBox";
            this.m_latitudeTextBox.Size = new System.Drawing.Size(137, 20);
            this.m_latitudeTextBox.TabIndex = 7;
            //
            // m_geohashTextBox
            //
            this.m_geohashTextBox.Location = new System.Drawing.Point(360, 43);
            this.m_geohashTextBox.Name = "m_geohashTextBox";
            this.m_geohashTextBox.Size = new System.Drawing.Size(155, 20);
            this.m_geohashTextBox.TabIndex = 8;
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(96, 83);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 10;
            this.button1.Text = "Convert ->";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnConvertDMS);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(229, 83);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(85, 23);
            this.button2.TabIndex = 11;
            this.button2.Text = "<- Convert ->";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnConvert);
            //
            // button3
            //
            this.button3.Location = new System.Drawing.Point(400, 83);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 12;
            this.button3.Text = "<- Convert";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.OnConvertGeohash);
            //
            // button4
            //
            this.button4.Location = new System.Drawing.Point(541, 4);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(75, 23);
            this.button4.TabIndex = 13;
            this.button4.Text = "Validate";
            this.button4.UseVisualStyleBackColor = true;
            this.button4.Click += new System.EventHandler(this.OnValidate);
            //
            // m_comboBox
            //
            this.m_comboBox.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.m_comboBox.FormattingEnabled = true;
            this.m_comboBox.Items.AddRange(new object[] {
            "Geohash",
            "GARS",
            "Georef"});
            this.m_comboBox.Location = new System.Drawing.Point(376, 4);
            this.m_comboBox.Name = "m_comboBox";
            this.m_comboBox.Size = new System.Drawing.Size(121, 21);
            this.m_comboBox.TabIndex = 14;
            this.m_toolTip.SetToolTip(this.m_comboBox, "Select the reference system");
            //
            // MiscPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_comboBox);
            this.Controls.Add(this.button4);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_geohashTextBox);
            this.Controls.Add(this.m_latitudeTextBox);
            this.Controls.Add(this.m_LongitudeTextBox);
            this.Controls.Add(this.m_latitudeDMSTextBox);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.m_longitudeDMSTextBox);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Name = "MiscPanel";
            this.Size = new System.Drawing.Size(977, 389);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ToolTip m_toolTip;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox m_longitudeDMSTextBox;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox m_latitudeDMSTextBox;
        private System.Windows.Forms.TextBox m_LongitudeTextBox;
        private System.Windows.Forms.TextBox m_latitudeTextBox;
        private System.Windows.Forms.TextBox m_geohashTextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.ComboBox m_comboBox;
    }
}
