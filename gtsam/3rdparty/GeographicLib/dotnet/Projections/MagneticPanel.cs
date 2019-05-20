/**
 * \file NETGeographicLib\MagneticPanel.cs
 * \brief NETGeographicLib.MagneticModel example
 *
 * NETGeographicLib.MagneticModel and
 * NETGeographicLib.MagneticCircle example.
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
    public partial class MagneticPanel : UserControl
    {
        string m_path, m_name;
        MagneticModel m_magnetic = null;

        public MagneticPanel()
        {
            InitializeComponent();
        }

        private void OnSelectMagneticModel(object sender, EventArgs e)
        {
            OpenFileDialog dlg = new OpenFileDialog();
            dlg.Filter = "Magnetic Model (*.wmm)|*.wmm";
            dlg.DefaultExt = "wmm";

            if (dlg.ShowDialog() == DialogResult.Cancel) return;

            m_path = dlg.FileName.Substring(0, dlg.FileName.LastIndexOf('\\')).Replace('\\', '/');
            int length = dlg.FileName.LastIndexOf('.') - dlg.FileName.LastIndexOf('\\') - 1;
            m_name = dlg.FileName.Substring(dlg.FileName.LastIndexOf('\\') + 1, length);

            try
            {
                m_magnetic = new MagneticModel(m_name, m_path);
                m_magneticModelNameTextBox.Text = dlg.FileName;
                m_nameTextBox.Text = m_magnetic.MagneticModelName;
                m_descriptionTextBox.Text = m_magnetic.Description;
                m_dateTextBox.Text = m_magnetic.DateTime;
                m_updateButton.Enabled = true;
                m_magCircButton.Enabled = true;
                m_validateButton.Enabled = true;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

        }

        private void OnUpdate(object sender, EventArgs e)
        {
            try
            {
                double time = Double.Parse(m_timeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double alt = Double.Parse(m_altitudeTextBox.Text);

                double bx, by, bz, bxt, byt, bzt;
                m_magnetic.Field(time, lat, lon, alt, out bx, out by, out bz, out bxt, out byt, out bzt);
                m_mfXTextBox.Text = bx.ToString();
                m_mfYTextBox.Text = by.ToString();
                m_mfZTextBox.Text = bz.ToString();
                m_mfdXTextBox.Text = bxt.ToString();
                m_mfdYTextBox.Text = byt.ToString();
                m_mfdZTextBox.Text = bzt.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnMagneticCircle(object sender, EventArgs e)
        {
            try
            {
                double time = Double.Parse(m_timeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double alt = Double.Parse(m_altitudeTextBox.Text);

                double bx, by, bz, bxt, byt, bzt;
                MagneticCircle mc = m_magnetic.Circle(time, lat, alt);
                mc.Field(lon, out bx, out by, out bz, out bxt, out byt, out bzt);
                m_mfXTextBox.Text = bx.ToString();
                m_mfYTextBox.Text = by.ToString();
                m_mfZTextBox.Text = bz.ToString();
                m_mfdXTextBox.Text = bxt.ToString();
                m_mfdYTextBox.Text = byt.ToString();
                m_mfdZTextBox.Text = bzt.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                double bx, by, bz, bxt, byt, bzt, x, y, z;
                double time = 1.0;
                double lon = -86.0;
                double lat = 32.0;
                double alt = 30.0;

                MagneticModel mag = new MagneticModel(m_name, m_path, new Geocentric());
                mag = new MagneticModel(m_name, m_path);
                mag.Field(time, lat, lon, alt, out bx, out by, out bz, out bxt, out byt, out bzt);
                mag.Field(time, lat, lon, alt, out x, out y, out z);
                if (bx != x || by != y || bz != z)
                    throw new Exception("Error in MagneticField.Field");
                MagneticCircle mc = mag.Circle(time, lat, alt);
                mc.Field(lon, out bx, out by, out bz);
                mc.Field(lon, out x, out y, out z, out bxt, out byt, out bzt);
                if (bx != x || by != y || bz != z )
                    throw new Exception("Error in MagneticCircle.Field (2)");

                double dtest = Utility.FractionalYear("2015.34");
                dtest = Utility.FractionalYear("2015-07-31");

                MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
