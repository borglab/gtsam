namespace Projections
{
    partial class AccumPanel
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
            this.m_inputTextBox = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.m_accumTtextBox = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.button4 = new System.Windows.Forms.Button();
            this.m_equalsCheckBox = new System.Windows.Forms.CheckBox();
            this.m_testTextBox = new System.Windows.Forms.TextBox();
            this.m_lessThanCheckBox = new System.Windows.Forms.CheckBox();
            this.m_greaterTanCheckBox = new System.Windows.Forms.CheckBox();
            this.SuspendLayout();
            //
            // label1
            //
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(4, 8);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(31, 13);
            this.label1.TabIndex = 0;
            this.label1.Text = "Input";
            //
            // m_inputTextBox
            //
            this.m_inputTextBox.Location = new System.Drawing.Point(109, 4);
            this.m_inputTextBox.Name = "m_inputTextBox";
            this.m_inputTextBox.Size = new System.Drawing.Size(202, 20);
            this.m_inputTextBox.TabIndex = 1;
            //
            // label2
            //
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(4, 35);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(99, 13);
            this.label2.TabIndex = 2;
            this.label2.Text = "Accumulated Value";
            //
            // m_accumTtextBox
            //
            this.m_accumTtextBox.Location = new System.Drawing.Point(109, 31);
            this.m_accumTtextBox.Name = "m_accumTtextBox";
            this.m_accumTtextBox.ReadOnly = true;
            this.m_accumTtextBox.Size = new System.Drawing.Size(202, 20);
            this.m_accumTtextBox.TabIndex = 3;
            //
            // button1
            //
            this.button1.Location = new System.Drawing.Point(317, 3);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 4;
            this.button1.Text = "Add";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.OnAdd);
            //
            // button2
            //
            this.button2.Location = new System.Drawing.Point(7, 64);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 5;
            this.button2.Text = "Reset";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.OnReset);
            //
            // button3
            //
            this.button3.Location = new System.Drawing.Point(318, 30);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 6;
            this.button3.Text = "Multiply";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.OnMultiply);
            //
            // button4
            //
            this.button4.Location = new System.Drawing.Point(405, 32);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(75, 23);
            this.button4.TabIndex = 7;
            this.button4.Text = "Test";
            this.button4.UseVisualStyleBackColor = true;
            this.button4.Click += new System.EventHandler(this.OnTest);
            //
            // m_equalsCheckBox
            //
            this.m_equalsCheckBox.AutoSize = true;
            this.m_equalsCheckBox.Location = new System.Drawing.Point(405, 69);
            this.m_equalsCheckBox.Name = "m_equalsCheckBox";
            this.m_equalsCheckBox.Size = new System.Drawing.Size(58, 17);
            this.m_equalsCheckBox.TabIndex = 8;
            this.m_equalsCheckBox.Text = "Equals";
            this.m_equalsCheckBox.UseVisualStyleBackColor = true;
            //
            // m_testTextBox
            //
            this.m_testTextBox.Location = new System.Drawing.Point(405, 4);
            this.m_testTextBox.Name = "m_testTextBox";
            this.m_testTextBox.Size = new System.Drawing.Size(139, 20);
            this.m_testTextBox.TabIndex = 10;
            //
            // m_lessThanCheckBox
            //
            this.m_lessThanCheckBox.AutoSize = true;
            this.m_lessThanCheckBox.Location = new System.Drawing.Point(405, 93);
            this.m_lessThanCheckBox.Name = "m_lessThanCheckBox";
            this.m_lessThanCheckBox.Size = new System.Drawing.Size(76, 17);
            this.m_lessThanCheckBox.TabIndex = 11;
            this.m_lessThanCheckBox.Text = "Less Than";
            this.m_lessThanCheckBox.UseVisualStyleBackColor = true;
            //
            // m_greaterTanCheckBox
            //
            this.m_greaterTanCheckBox.AutoSize = true;
            this.m_greaterTanCheckBox.Location = new System.Drawing.Point(405, 117);
            this.m_greaterTanCheckBox.Name = "m_greaterTanCheckBox";
            this.m_greaterTanCheckBox.Size = new System.Drawing.Size(86, 17);
            this.m_greaterTanCheckBox.TabIndex = 12;
            this.m_greaterTanCheckBox.Text = "GreaterThan";
            this.m_greaterTanCheckBox.UseVisualStyleBackColor = true;
            //
            // AccumPanel
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.m_greaterTanCheckBox);
            this.Controls.Add(this.m_lessThanCheckBox);
            this.Controls.Add(this.m_testTextBox);
            this.Controls.Add(this.m_equalsCheckBox);
            this.Controls.Add(this.button4);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.m_accumTtextBox);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.m_inputTextBox);
            this.Controls.Add(this.label1);
            this.Name = "AccumPanel";
            this.Size = new System.Drawing.Size(969, 353);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox m_inputTextBox;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox m_accumTtextBox;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Button button2;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.CheckBox m_equalsCheckBox;
        private System.Windows.Forms.TextBox m_testTextBox;
        private System.Windows.Forms.CheckBox m_lessThanCheckBox;
        private System.Windows.Forms.CheckBox m_greaterTanCheckBox;
    }
}
