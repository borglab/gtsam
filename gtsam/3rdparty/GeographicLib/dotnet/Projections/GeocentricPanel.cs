/**
 * \file NETGeographicLib\GeocentricPanel.cs
 * \brief NETGeographicLib.Geocentric example
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
    public partial class GeocentricPanel : UserControl
    {
        enum Functions
        {
            Forward,
            Inverse
        };
        Functions m_function = Functions.Forward;
        Geocentric m_geocentric = null;

        public GeocentricPanel()
        {
            InitializeComponent();

            try
            {
                m_geocentric = new Geocentric();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            m_majorRadiusTextBox.Text = m_geocentric.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_geocentric.Flattening.ToString();
            m_functionComboBox.SelectedIndex = (int)m_function;
        }

        private void OnSetParameters(object sender, EventArgs e)
        {
            try
            {
                double a = Double.Parse( m_majorRadiusTextBox.Text );
                double f = Double.Parse(m_flatteningTextBox.Text);
                m_geocentric = new Geocentric(a, f);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Data entry error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
        }

        private void OnFunctionChanged(object sender, EventArgs e)
        {
            m_function = (Functions)m_functionComboBox.SelectedIndex;
            switch (m_function)
            {
                case Functions.Forward:
                    m_latitudeTextBox.ReadOnly = m_longitudeTextBox.ReadOnly = m_altitudeTextBox.ReadOnly = false;
                    m_XTextBox.ReadOnly = m_YTextBox.ReadOnly = m_ZTextBox.ReadOnly = true;
                    break;
                case Functions.Inverse:
                    m_latitudeTextBox.ReadOnly = m_longitudeTextBox.ReadOnly = m_altitudeTextBox.ReadOnly = true;
                    m_XTextBox.ReadOnly = m_YTextBox.ReadOnly = m_ZTextBox.ReadOnly = false;
                    break;
            }
        }

        private void OnConvert(object sender, EventArgs e)
        {
            try
            {
                double lat, lon, alt, x, y, z;
                double[,] rot = null;
                switch (m_function)
                {
                    case Functions.Forward:
                        lat = Double.Parse(m_latitudeTextBox.Text);
                        lon = Double.Parse(m_longitudeTextBox.Text);
                        alt = Double.Parse(m_altitudeTextBox.Text);
                        m_geocentric.Forward(lat, lon, alt, out x, out y, out z, out rot);
                        m_XTextBox.Text = x.ToString();
                        m_YTextBox.Text = y.ToString();
                        m_ZTextBox.Text = z.ToString();
                        break;
                    case Functions.Inverse:
                        x = Double.Parse(m_XTextBox.Text);
                        y = Double.Parse(m_YTextBox.Text);
                        z = Double.Parse(m_ZTextBox.Text);
                        m_geocentric.Reverse(x, y, z, out lat, out lon, out alt, out rot);
                        m_latitudeTextBox.Text = lat.ToString();
                        m_longitudeTextBox.Text = lon.ToString();
                        m_altitudeTextBox.Text = alt.ToString();
                        break;
                }
                m_textBox00.Text = rot[0, 0].ToString();
                m_textBox01.Text = rot[0, 1].ToString();
                m_textBox02.Text = rot[0, 2].ToString();
                m_textBox10.Text = rot[1, 0].ToString();
                m_textBox11.Text = rot[1, 1].ToString();
                m_textBox12.Text = rot[1, 2].ToString();
                m_textBox20.Text = rot[2, 0].ToString();
                m_textBox21.Text = rot[2, 1].ToString();
                m_textBox22.Text = rot[2, 2].ToString();
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
                Geocentric g = new Geocentric();
                string test = g.ToString();
                g = new Geocentric(g.MajorRadius, g.Flattening);
                double x, y, z, lat, lon, alt, tx, ty, tz;
                double[,] rot;
                g.Forward(32.0, -86.0, 45.0, out x, out y, out z, out rot);
                g.Forward(32.0, -86.0, 45.0, out tx, out ty, out tz);
                if (x != tx || y != ty || z != tz)
                    throw new Exception("Error in Forward");
                g.Reverse(x, y, z, out lat, out lon, out alt, out rot);
                g.Reverse(x, y, z, out tx, out ty, out tz);
                if ( lat != tx || lon != ty || alt != tz )
                    throw new Exception("Error in Reverse");
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error detected", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
