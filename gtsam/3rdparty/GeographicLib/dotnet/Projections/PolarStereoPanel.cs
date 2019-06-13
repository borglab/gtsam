/**
 * \file NETGeographicLib\PolarStereoPanel.cs
 * \brief NETGeographicLib.PolarStereographic example
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
    public partial class PolarStereoPanel : UserControl
    {
        PolarStereographic m_polar = null;

        public PolarStereoPanel()
        {
            InitializeComponent();
            try
            {
                m_polar = new PolarStereographic();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            m_majorRadiusTextBox.Text = m_polar.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_polar.Flattening.ToString();
            m_scaleTextBox.Text = m_polar.CentralScale.ToString();
            m_functionComboBox.SelectedIndex = 0;
        }

        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                double a = Double.Parse(m_majorRadiusTextBox.Text);
                double f = Double.Parse(m_flatteningTextBox.Text);
                double k = Double.Parse(m_scaleTextBox.Text);
                m_polar = new PolarStereographic(a, f, k);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnFunction(object sender, EventArgs e)
        {
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    m_latitudeTextBox.ReadOnly = m_longitudeTextBox.ReadOnly = false;
                    m_xTextBox.ReadOnly = m_yTextBox.ReadOnly = true;
                    break;
                case 1:
                    m_latitudeTextBox.ReadOnly = m_longitudeTextBox.ReadOnly = true;
                    m_xTextBox.ReadOnly = m_yTextBox.ReadOnly = false;
                    break;
            }
        }

        private void OnConvert(object sender, EventArgs e)
        {
            try
            {
                double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, gamma = 0.0, k = 0.0;
                switch (m_functionComboBox.SelectedIndex)
                {
                    case 0:
                        lat = Double.Parse(m_latitudeTextBox.Text);
                        lon = Double.Parse(m_longitudeTextBox.Text);
                        m_polar.Forward(m_northPoleCheckBox.Checked, lat, lon, out x, out y, out gamma, out k);
                        m_xTextBox.Text = x.ToString();
                        m_yTextBox.Text = y.ToString();
                        break;
                    case 1:
                        x = Double.Parse(m_xTextBox.Text);
                        y = Double.Parse(m_yTextBox.Text);
                        m_polar.Reverse(m_northPoleCheckBox.Checked, x, y, out lat, out lon, out gamma, out k);
                        m_latitudeTextBox.Text = lat.ToString();
                        m_longitudeTextBox.Text = lon.ToString();
                        break;
                }
                m_gammaTextBox.Text = gamma.ToString();
                m_kTextBox.Text = k.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                double lat, lon, x, y, x1, y1, gamma, k;
                PolarStereographic p = new PolarStereographic(m_polar.MajorRadius, m_polar.Flattening, m_polar.CentralScale);
                p.SetScale(60.0, 1.0);
                p = new PolarStereographic();
                p.Forward(true, 32.0, -86.0, out x, out y, out gamma, out k);
                p.Forward(true, 32.0, -86.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in Forward");
                p.Reverse(true, x, y, out lat, out lon, out gamma, out k);
                p.Reverse(true, x, y, out x1, out y1);
                if ( lat != x1 || lon != y1 )
                    throw new Exception("Error in Reverse");
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            MessageBox.Show("No errors detected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
