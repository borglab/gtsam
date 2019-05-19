/**
 * \file NETGeographicLib\LocalCartesianPanel.cs
 * \brief NETGeographicLib.LocalCartesian example
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
    public partial class LocalCartesianPanel : UserControl
    {
        LocalCartesian m_lc = null;
        enum Functions
        {
            Forward,
            Reverse
        };
        Functions m_function = Functions.Forward;

        public LocalCartesianPanel()
        {
            InitializeComponent();

            Geocentric g = new Geocentric();
            m_majorRadiusTextBox.Text = g.MajorRadius.ToString();
            m_flatteningTextBox.Text = g.Flattening.ToString();
            m_lc = new LocalCartesian(g);
            m_functionComboBox.SelectedIndex = (int)m_function;
        }

        private void OnSetEllipsoid(object sender, EventArgs e)
        {
            double a, f;
            try
            {
                a = Double.Parse(m_majorRadiusTextBox.Text);
                f = Double.Parse(m_flatteningTextBox.Text);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Data Entry Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            try
            {
                m_lc = new LocalCartesian(new Geocentric(a, f));
            }
            catch (GeographicErr err)
            {
                MessageBox.Show(err.Message, "GeographicLib error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnSetReference(object sender, EventArgs e)
        {
            double lat, lon, alt;
            try
            {
                lat = Double.Parse(m_latitudeTextBox.Text);
                lon = Double.Parse(m_longitudeTextBox.Text);
                alt = Double.Parse(m_altitudeTextBox.Text);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Data Entry Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            m_lc.Reset(lat, lon, alt);
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
                        lat = Double.Parse(m_latTextBox.Text);
                        lon = Double.Parse(m_lonTextBox.Text);
                        alt = Double.Parse(m_altTextBox.Text);
                        m_lc.Forward(lat, lon, alt, out x, out y, out z, out rot);
                        m_XTextBox.Text = x.ToString("0.00###");
                        m_YTextBox.Text = y.ToString("0.00###");
                        m_ZTextBox.Text = z.ToString("0.00###");
                        break;
                    case Functions.Reverse:
                        x = Double.Parse(m_XTextBox.Text);
                        y = Double.Parse(m_YTextBox.Text);
                        z = Double.Parse(m_ZTextBox.Text);
                        m_lc.Reverse(x, y, z, out lat, out lon, out alt, out rot);
                        m_latTextBox.Text = lat.ToString();
                        m_lonTextBox.Text = lon.ToString();
                        m_altTextBox.Text = alt.ToString();
                        break;
                }
                m_textBox00.Text = rot[0, 0].ToString("#.000000000000");
                m_textBox01.Text = rot[0, 1].ToString("#.000000000000");
                m_textBox02.Text = rot[0, 2].ToString("#.000000000000");
                m_textBox10.Text = rot[1, 0].ToString("#.000000000000");
                m_textBox11.Text = rot[1, 1].ToString("#.000000000000");
                m_textBox12.Text = rot[1, 2].ToString("#.000000000000");
                m_textBox20.Text = rot[2, 0].ToString("#.000000000000");
                m_textBox21.Text = rot[2, 1].ToString("#.000000000000");
                m_textBox22.Text = rot[2, 2].ToString("#.000000000000");
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnNewFunction(object sender, EventArgs e)
        {
            m_function = (Functions)m_functionComboBox.SelectedIndex;
            switch (m_function)
            {
                case Functions.Forward:
                    m_latTextBox.ReadOnly = m_lonTextBox.ReadOnly = m_altTextBox.ReadOnly = false;
                    m_XTextBox.ReadOnly = m_YTextBox.ReadOnly = m_ZTextBox.ReadOnly = true;
                    break;
                case Functions.Reverse:
                    m_latTextBox.ReadOnly = m_lonTextBox.ReadOnly = m_altTextBox.ReadOnly = true;
                    m_XTextBox.ReadOnly = m_YTextBox.ReadOnly = m_ZTextBox.ReadOnly = false;
                    break;
            }
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                LocalCartesian p = new LocalCartesian();
                p = new LocalCartesian(new Geocentric());
                p = new LocalCartesian(32.0, -86.0, 45.0);
                p = new LocalCartesian(32.0, -86.0, 45.0, new Geocentric());
                double x, y, z, x1, y1, z1;
                double[,] rot;
                p.Forward(32.0, -86.0, 45.0, out x, out y, out z, out rot);
                p.Forward(32.0, -86.0, 45.0, out x1, out y1, out z1);
                if (x != x1 || y != y1 || z != z1)
                    throw new Exception("Error in Forward");
                double lat, lon, alt;
                p.Reverse(x, y, z, out lat, out lon, out alt, out rot);
                p.Reverse(x, y, z, out x1, out y1, out z1);
                if (lat != x1 || lon != y1 || alt != z1)
                    throw new Exception("Error in Reverse");
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
