namespace Projections
{
    partial class Form1
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

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.m_tabControl = new System.Windows.Forms.TabControl();
            this.m_geocentricTabPage = new System.Windows.Forms.TabPage();
            this.m_geodesicTabPage = new System.Windows.Forms.TabPage();
            this.m_localCartesianPage = new System.Windows.Forms.TabPage();
            this.m_albersPage = new System.Windows.Forms.TabPage();
            this.m_typeIIProjections = new System.Windows.Forms.TabPage();
            this.m_typeIIIProjPage = new System.Windows.Forms.TabPage();
            this.m_polarStereoPage = new System.Windows.Forms.TabPage();
            this.m_sphericalPage = new System.Windows.Forms.TabPage();
            this.m_ellipticPage = new System.Windows.Forms.TabPage();
            this.m_ellipsoidPage = new System.Windows.Forms.TabPage();
            this.m_miscPage = new System.Windows.Forms.TabPage();
            this.m_geoidPage = new System.Windows.Forms.TabPage();
            this.m_gravityPage = new System.Windows.Forms.TabPage();
            this.m_magneticPage = new System.Windows.Forms.TabPage();
            this.m_polyPage = new System.Windows.Forms.TabPage();
            this.m_accumPage = new System.Windows.Forms.TabPage();
            this.m_rhumbTabPage = new System.Windows.Forms.TabPage();
            this.m_tabControl.SuspendLayout();
            this.SuspendLayout();
            //
            // m_tabControl
            //
            this.m_tabControl.Controls.Add(this.m_geocentricTabPage);
            this.m_tabControl.Controls.Add(this.m_geodesicTabPage);
            this.m_tabControl.Controls.Add(this.m_localCartesianPage);
            this.m_tabControl.Controls.Add(this.m_albersPage);
            this.m_tabControl.Controls.Add(this.m_typeIIProjections);
            this.m_tabControl.Controls.Add(this.m_typeIIIProjPage);
            this.m_tabControl.Controls.Add(this.m_polarStereoPage);
            this.m_tabControl.Controls.Add(this.m_sphericalPage);
            this.m_tabControl.Controls.Add(this.m_ellipticPage);
            this.m_tabControl.Controls.Add(this.m_ellipsoidPage);
            this.m_tabControl.Controls.Add(this.m_miscPage);
            this.m_tabControl.Controls.Add(this.m_geoidPage);
            this.m_tabControl.Controls.Add(this.m_gravityPage);
            this.m_tabControl.Controls.Add(this.m_magneticPage);
            this.m_tabControl.Controls.Add(this.m_polyPage);
            this.m_tabControl.Controls.Add(this.m_accumPage);
            this.m_tabControl.Controls.Add(this.m_rhumbTabPage);
            this.m_tabControl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.m_tabControl.Location = new System.Drawing.Point(0, 0);
            this.m_tabControl.Name = "m_tabControl";
            this.m_tabControl.SelectedIndex = 0;
            this.m_tabControl.ShowToolTips = true;
            this.m_tabControl.Size = new System.Drawing.Size(944, 262);
            this.m_tabControl.TabIndex = 0;
            //
            // m_geocentricTabPage
            //
            this.m_geocentricTabPage.Location = new System.Drawing.Point(4, 22);
            this.m_geocentricTabPage.Name = "m_geocentricTabPage";
            this.m_geocentricTabPage.Size = new System.Drawing.Size(936, 236);
            this.m_geocentricTabPage.TabIndex = 1;
            this.m_geocentricTabPage.Text = "Geocentric";
            this.m_geocentricTabPage.UseVisualStyleBackColor = true;
            //
            // m_geodesicTabPage
            //
            this.m_geodesicTabPage.Location = new System.Drawing.Point(4, 22);
            this.m_geodesicTabPage.Name = "m_geodesicTabPage";
            this.m_geodesicTabPage.Padding = new System.Windows.Forms.Padding(3);
            this.m_geodesicTabPage.Size = new System.Drawing.Size(936, 236);
            this.m_geodesicTabPage.TabIndex = 0;
            this.m_geodesicTabPage.Text = "Geodesic";
            this.m_geodesicTabPage.ToolTipText = "Geodesic, GeodesicExact, GeodesicLine, & GeodesicLineExact";
            this.m_geodesicTabPage.UseVisualStyleBackColor = true;
            //
            // m_localCartesianPage
            //
            this.m_localCartesianPage.Location = new System.Drawing.Point(4, 22);
            this.m_localCartesianPage.Name = "m_localCartesianPage";
            this.m_localCartesianPage.Size = new System.Drawing.Size(936, 236);
            this.m_localCartesianPage.TabIndex = 2;
            this.m_localCartesianPage.Text = "LocalCartesian";
            this.m_localCartesianPage.UseVisualStyleBackColor = true;
            //
            // m_albersPage
            //
            this.m_albersPage.Location = new System.Drawing.Point(4, 22);
            this.m_albersPage.Name = "m_albersPage";
            this.m_albersPage.Size = new System.Drawing.Size(936, 236);
            this.m_albersPage.TabIndex = 3;
            this.m_albersPage.Text = "Type I Projections";
            this.m_albersPage.ToolTipText = "Albers Equal Area, Lambert Conformal Conic, Transverse Mercator, and Transverse M" +
                "ercator Exact Projections";
            this.m_albersPage.UseVisualStyleBackColor = true;
            //
            // m_typeIIProjections
            //
            this.m_typeIIProjections.Location = new System.Drawing.Point(4, 22);
            this.m_typeIIProjections.Name = "m_typeIIProjections";
            this.m_typeIIProjections.Size = new System.Drawing.Size(936, 236);
            this.m_typeIIProjections.TabIndex = 4;
            this.m_typeIIProjections.Text = "Type II Projections";
            this.m_typeIIProjections.ToolTipText = "Azimuth Equidistant, Cassini Soldner, and Gnomonic Projections";
            this.m_typeIIProjections.UseVisualStyleBackColor = true;
            //
            // m_typeIIIProjPage
            //
            this.m_typeIIIProjPage.Location = new System.Drawing.Point(4, 22);
            this.m_typeIIIProjPage.Name = "m_typeIIIProjPage";
            this.m_typeIIIProjPage.Size = new System.Drawing.Size(936, 236);
            this.m_typeIIIProjPage.TabIndex = 11;
            this.m_typeIIIProjPage.Text = "Type III Projections";
            this.m_typeIIIProjPage.ToolTipText = "MGRS/OSGB/UTMUPS";
            this.m_typeIIIProjPage.UseVisualStyleBackColor = true;
            //
            // m_polarStereoPage
            //
            this.m_polarStereoPage.Location = new System.Drawing.Point(4, 22);
            this.m_polarStereoPage.Name = "m_polarStereoPage";
            this.m_polarStereoPage.Size = new System.Drawing.Size(936, 236);
            this.m_polarStereoPage.TabIndex = 5;
            this.m_polarStereoPage.Text = "Polar Stereographic";
            this.m_polarStereoPage.UseVisualStyleBackColor = true;
            //
            // m_sphericalPage
            //
            this.m_sphericalPage.Location = new System.Drawing.Point(4, 22);
            this.m_sphericalPage.Name = "m_sphericalPage";
            this.m_sphericalPage.Size = new System.Drawing.Size(936, 236);
            this.m_sphericalPage.TabIndex = 6;
            this.m_sphericalPage.Text = "Spherical Harmonics";
            this.m_sphericalPage.ToolTipText = "Spherical Harmonic, Spherical Harmonic 1, and Spherical Harmonic 2";
            this.m_sphericalPage.UseVisualStyleBackColor = true;
            //
            // m_ellipticPage
            //
            this.m_ellipticPage.Location = new System.Drawing.Point(4, 22);
            this.m_ellipticPage.Name = "m_ellipticPage";
            this.m_ellipticPage.Size = new System.Drawing.Size(936, 236);
            this.m_ellipticPage.TabIndex = 7;
            this.m_ellipticPage.Text = "Elliptic Function";
            this.m_ellipticPage.UseVisualStyleBackColor = true;
            //
            // m_ellipsoidPage
            //
            this.m_ellipsoidPage.Location = new System.Drawing.Point(4, 22);
            this.m_ellipsoidPage.Name = "m_ellipsoidPage";
            this.m_ellipsoidPage.Size = new System.Drawing.Size(936, 236);
            this.m_ellipsoidPage.TabIndex = 8;
            this.m_ellipsoidPage.Text = "Ellipsoid";
            this.m_ellipsoidPage.UseVisualStyleBackColor = true;
            //
            // m_miscPage
            //
            this.m_miscPage.Location = new System.Drawing.Point(4, 22);
            this.m_miscPage.Name = "m_miscPage";
            this.m_miscPage.Size = new System.Drawing.Size(936, 236);
            this.m_miscPage.TabIndex = 9;
            this.m_miscPage.Text = "Miscellaneous";
            this.m_miscPage.ToolTipText = "DDS/Geohash";
            this.m_miscPage.UseVisualStyleBackColor = true;
            //
            // m_geoidPage
            //
            this.m_geoidPage.Location = new System.Drawing.Point(4, 22);
            this.m_geoidPage.Name = "m_geoidPage";
            this.m_geoidPage.Size = new System.Drawing.Size(936, 236);
            this.m_geoidPage.TabIndex = 10;
            this.m_geoidPage.Text = "Geoid";
            this.m_geoidPage.UseVisualStyleBackColor = true;
            //
            // m_gravityPage
            //
            this.m_gravityPage.Location = new System.Drawing.Point(4, 22);
            this.m_gravityPage.Name = "m_gravityPage";
            this.m_gravityPage.Size = new System.Drawing.Size(936, 236);
            this.m_gravityPage.TabIndex = 12;
            this.m_gravityPage.Text = "Gravity";
            this.m_gravityPage.ToolTipText = "GravityModel/GravityCircle/NormalGravity";
            this.m_gravityPage.UseVisualStyleBackColor = true;
            //
            // m_magneticPage
            //
            this.m_magneticPage.Location = new System.Drawing.Point(4, 22);
            this.m_magneticPage.Name = "m_magneticPage";
            this.m_magneticPage.Size = new System.Drawing.Size(936, 236);
            this.m_magneticPage.TabIndex = 13;
            this.m_magneticPage.Text = "Magnetic";
            this.m_magneticPage.ToolTipText = "MagneticModel/MagneticCircle";
            this.m_magneticPage.UseVisualStyleBackColor = true;
            //
            // m_polyPage
            //
            this.m_polyPage.Location = new System.Drawing.Point(4, 22);
            this.m_polyPage.Name = "m_polyPage";
            this.m_polyPage.Size = new System.Drawing.Size(936, 236);
            this.m_polyPage.TabIndex = 14;
            this.m_polyPage.Text = "PolygonArea";
            this.m_polyPage.UseVisualStyleBackColor = true;
            //
            // m_accumPage
            //
            this.m_accumPage.Location = new System.Drawing.Point(4, 22);
            this.m_accumPage.Name = "m_accumPage";
            this.m_accumPage.Size = new System.Drawing.Size(936, 236);
            this.m_accumPage.TabIndex = 15;
            this.m_accumPage.Text = "Accumulator";
            this.m_accumPage.UseVisualStyleBackColor = true;
            //
            // m_rhumbTabPage
            //
            this.m_rhumbTabPage.Location = new System.Drawing.Point(4, 22);
            this.m_rhumbTabPage.Name = "m_rhumbTabPage";
            this.m_rhumbTabPage.Padding = new System.Windows.Forms.Padding(3);
            this.m_rhumbTabPage.Size = new System.Drawing.Size(936, 236);
            this.m_rhumbTabPage.TabIndex = 16;
            this.m_rhumbTabPage.Text = "Rhumb";
            this.m_rhumbTabPage.UseVisualStyleBackColor = true;
            //
            // Form1
            //
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(944, 262);
            this.Controls.Add(this.m_tabControl);
            this.Name = "Form1";
            this.Text = "Projections";
            this.m_tabControl.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.TabControl m_tabControl;
        private System.Windows.Forms.TabPage m_geodesicTabPage;
        private System.Windows.Forms.TabPage m_geocentricTabPage;
        private System.Windows.Forms.TabPage m_localCartesianPage;
        private System.Windows.Forms.TabPage m_albersPage;
        private System.Windows.Forms.TabPage m_typeIIProjections;
        private System.Windows.Forms.TabPage m_polarStereoPage;
        private System.Windows.Forms.TabPage m_sphericalPage;
        private System.Windows.Forms.TabPage m_ellipticPage;
        private System.Windows.Forms.TabPage m_ellipsoidPage;
        private System.Windows.Forms.TabPage m_miscPage;
        private System.Windows.Forms.TabPage m_geoidPage;
        private System.Windows.Forms.TabPage m_typeIIIProjPage;
        private System.Windows.Forms.TabPage m_gravityPage;
        private System.Windows.Forms.TabPage m_magneticPage;
        private System.Windows.Forms.TabPage m_polyPage;
        private System.Windows.Forms.TabPage m_accumPage;
        private System.Windows.Forms.TabPage m_rhumbTabPage;

    }
}
