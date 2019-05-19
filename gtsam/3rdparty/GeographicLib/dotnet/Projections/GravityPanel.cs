/**
 * \file NETGeographicLib\GravityPanel.cs
 * \brief NETGeographicLib.GravityModel example
 *
 * NETGeographicLib.GravityModel,
 * NETGeographicLib.NormalGravity, and
 * NETGeographicLib.GravityCircle example.
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using NETGeographicLib;

namespace Projections
{
    public partial class GravityPanel : UserControl
    {
        GravityModel m_gm = null;
        string m_path;
        string m_name;
        public GravityPanel()
        {
            InitializeComponent();
        }

        private void OnSelectGravityModel(object sender, EventArgs e)
        {
            OpenFileDialog dlg = new OpenFileDialog();
            dlg.Filter = "Gravity Model (*.egm)|*.egm";
            dlg.DefaultExt = "egm";

            if (dlg.ShowDialog() == DialogResult.Cancel) return;

            m_path = dlg.FileName.Substring(0, dlg.FileName.LastIndexOf('\\')).Replace('\\', '/');
            int length = dlg.FileName.LastIndexOf('.') - dlg.FileName.LastIndexOf('\\') - 1;
            m_name = dlg.FileName.Substring(dlg.FileName.LastIndexOf('\\') + 1, length);

            try
            {
                m_gm = new GravityModel(m_name, m_path);
                m_gravityModelNameTextBox.Text = dlg.FileName;
                m_nameTextBox.Text = m_gm.GravityModelName;
                m_descriptionTextBox.Text = m_gm.Description;
                m_dateTextBox.Text = m_gm.DateTime;
                m_updateButton.Enabled = true;
                m_normGravButton.Enabled = true;
                m_GravityCircleButton.Enabled = true;
                m_validateButton.Enabled = true;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnUpdate(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            try
            {
                double lon = Double.Parse(m_longitudetextBoxT.Text);
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double alt = Double.Parse(m_altitudeTextBox.Text);

                double gx, gy, gz;
                m_gm.Gravity(lat, lon, alt, out gx, out gy, out gz);
                m_accelXTextBox.Text = gx.ToString();
                m_accelYTextBox.Text = gy.ToString();
                m_accelZTextBox.Text = gz.ToString();
                m_geoidTextBox.Text = m_gm.GeoidHeight(lat, lon).ToString();
                Cursor = Cursors.Default;
            }
            catch (Exception xcpt)
            {
                Cursor = Cursors.Default;
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnNormGravity(object sender, EventArgs e)
        {
            try
            {
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double alt = Double.Parse(m_altitudeTextBox.Text);

                double gx, gz;
                NormalGravity ng = m_gm.ReferenceEllipsoid();
                ng.Gravity(lat, alt, out gx, out gz);
                m_accelXTextBox.Text = gx.ToString();
                m_accelYTextBox.Text = "0.0";
                m_accelZTextBox.Text = gz.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

        }

        private void OnGravityCircle(object sender, EventArgs e)
        {
            Cursor = Cursors.WaitCursor;
            try
            {
                double lon = Double.Parse(m_longitudetextBoxT.Text);
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double alt = Double.Parse(m_altitudeTextBox.Text);

                double gx, gy, gz;
                GravityCircle gc = m_gm.Circle(lat, alt, GravityModel.Mask.GEOID_HEIGHT|GravityModel.Mask.GRAVITY);
                gc.Gravity(lon, out gx, out gy, out gz);
                m_accelXTextBox.Text = gx.ToString();
                m_accelYTextBox.Text = gy.ToString();
                m_accelZTextBox.Text = gz.ToString();
                if (alt != 0.0)
                    MessageBox.Show("Geoid height cannot be calculated with GravityCircle if altitude is not 0", "", MessageBoxButtons.OK, MessageBoxIcon.Information);
                else
                    m_geoidTextBox.Text = gc.GeoidHeight(lon).ToString();
                Cursor = Cursors.Default;
            }
            catch (Exception xcpt)
            {
                Cursor = Cursors.Default;
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                double lon = -86.0;
                double lat = 32.0;
                double alt = 0.0;

                double x, y, z;
                GravityModel gm = new GravityModel(m_name,m_path);
                gm.Disturbance( lat, lon, alt, out x, out y, out z);
                gm.GeoidHeight(lat,lon);
                gm.Gravity(lat,lon,alt,out x,out y, out z);
                gm.Phi(5000000.0,5000000.0,out x,out y);
                gm.SphericalAnomaly(lat,lon,alt,out x, out y, out z);
                gm.T(5000000.0,5000000.0,5000000.0);
                gm.U(5000000.0,5000000.0,5000000.0,out x,out y,out z);
                gm.V(5000000.0,5000000.0,5000000.0,out x,out y,out z);
                gm.W(5000000.0,5000000.0,5000000.0,out x,out y,out z);
                NormalGravity ng = new NormalGravity(NormalGravity.StandardModels.GRS80);
                ng = new NormalGravity( NormalGravity.StandardModels.WGS84);
                ng = new NormalGravity(6378137.0,3.986005e+14,7.292115147e-5,-1.0,1.08263e-3);
                ng = gm.ReferenceEllipsoid();
                ng.DynamicalFormFactor(1);
                Geocentric geo = ng.Earth();
                ng.Gravity(lat,alt,out x, out z);
                ng.Phi(5000000.0,5000000.0,out x,out y);
                ng.SurfaceGravity(lat);
                ng.U(5000000.0,5000000.0,5000000.0,out x, out y, out z);
                ng.V0(5000000.0,5000000.0,5000000.0,out x, out y, out z);
                GravityCircle gc = gm.Circle(lat,0.0,GravityModel.Mask.ALL);
                gc.Capabilities();
                gc.Capabilities(GravityModel.Mask.GRAVITY);
                gc.Disturbance(lon, out x, out y, out z);
                gc.GeoidHeight(lon);
                gc.Gravity(lon, out x, out y, out z);
                gc.SphericalAnomaly(lon, out x, out y, out z);
                gc.T(lon);
                gc.V(lon, out x, out y, out z);
                gc.W(lon, out x, out y, out z);
                MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
