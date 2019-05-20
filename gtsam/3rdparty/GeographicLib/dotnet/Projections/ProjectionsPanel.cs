/**
 * \file NETGeographicLib\ProjectionsPanel.cs
 * \brief NETGeographicLib projection example
 *
 * NETGeographicLib.AzimuthalEquidistant,
 * NETGeographicLib.CassiniSoldner, and
 * NETGeographicLib.Gnomonic example.
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
    public partial class ProjectionsPanel : UserControl
    {
        enum ProjectionTypes
        {
            AzimuthalEquidistant = 0,
            CassiniSoldner = 1,
            Gnomonic = 2
        }
        ProjectionTypes m_type;
        AzimuthalEquidistant m_azimuthal = null;
        CassiniSoldner m_cassini = null;
        Gnomonic m_gnomonic = null;
        Geodesic m_geodesic = null;

        public ProjectionsPanel()
        {
            InitializeComponent();
            try
            {
                m_geodesic = new Geodesic();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            m_majorRadiusTextBox.Text = m_geodesic.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_geodesic.Flattening.ToString();
            m_lat0TextBox.Text = m_lon0TextBox.Text = "0";
            m_projectionComboBox.SelectedIndex = 0;
            m_functionComboBox.SelectedIndex = 0;
        }

        private void OnProjectectionType(object sender, EventArgs e)
        {
            m_type = (ProjectionTypes)m_projectionComboBox.SelectedIndex;
            switch (m_type)
            {
                case ProjectionTypes.AzimuthalEquidistant:
                    m_azimuthal = new AzimuthalEquidistant(m_geodesic);
                    break;
                case ProjectionTypes.CassiniSoldner:
                    double lat0 = Double.Parse( m_lat0TextBox.Text );
                    double lon0 = Double.Parse( m_lon0TextBox.Text );
                    m_cassini = new CassiniSoldner(lat0, lon0, m_geodesic);
                    break;
                case ProjectionTypes.Gnomonic:
                    m_gnomonic = new Gnomonic(m_geodesic);
                    break;
            }
        }

        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                double a = Double.Parse(m_majorRadiusTextBox.Text);
                double f = Double.Parse(m_flatteningTextBox.Text);
                m_geodesic = new Geodesic(a, f);
            }
            catch ( Exception xcpt)
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
                switch (m_type)
                {
                    case ProjectionTypes.AzimuthalEquidistant:
                        ConvertAzimuthalEquidistant();
                        break;
                    case ProjectionTypes.CassiniSoldner:
                        ConvertCassiniSoldner();
                        break;
                    case ProjectionTypes.Gnomonic:
                        ConvertGnomonic();
                        break;
                }
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void ConvertAzimuthalEquidistant()
        {
            double lat0 = Double.Parse(m_lat0TextBox.Text);
            double lon0 = Double.Parse(m_lon0TextBox.Text);
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, azi = 0.0, rk = 0.0;
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_azimuthal.Forward(lat0, lon0, lat, lon, out x, out y, out azi, out rk);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_azimuthal.Reverse(lat0, lon0, x, y, out lat, out lon, out azi, out rk);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_azimuthTextBox.Text = azi.ToString();
            m_scaleTextBox.Text = rk.ToString();
        }

        private void ConvertCassiniSoldner()
        {
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, azi = 0.0, rk = 0.0;
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_cassini.Forward(lat, lon, out x, out y, out azi, out rk);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_cassini.Reverse(x, y, out lat, out lon, out azi, out rk);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_azimuthTextBox.Text = azi.ToString();
            m_scaleTextBox.Text = rk.ToString();
        }

        private void ConvertGnomonic()
        {
            double lat0 = Double.Parse(m_lat0TextBox.Text);
            double lon0 = Double.Parse(m_lon0TextBox.Text);
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, azi = 0.0, rk = 0.0;
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_gnomonic.Forward(lat0, lon0, lat, lon, out x, out y, out azi, out rk);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_gnomonic.Reverse(lat0, lon0, x, y, out lat, out lon, out azi, out rk);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_azimuthTextBox.Text = azi.ToString();
            m_scaleTextBox.Text = rk.ToString();
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, x1 = 0.0, y1 = 0.0, azi = 0.0, rk = 0.0;
                AzimuthalEquidistant azimuthal = new AzimuthalEquidistant(m_geodesic);
                azimuthal = new AzimuthalEquidistant();
                azimuthal.Forward(32.0, -86.0, 33.0, -87.0, out x, out y, out azi, out rk);
                azimuthal.Forward(32.0, -86.0, 33.0, -87.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in AzimuthalEquidistant.Forward");
                azimuthal.Reverse(32.0, -86.0, x, y, out lat, out lon, out azi, out rk);
                azimuthal.Reverse(32.0, -86.0, x, y, out x1, out y1);
                if ( x1 != lat || y1 != lon )
                    throw new Exception("Error in AzimuthalEquidistant.Reverse");
                CassiniSoldner cassini = new CassiniSoldner(32.0, -86.0, m_geodesic);
                cassini = new CassiniSoldner(32.0, -86.0);
                cassini.Reset(31.0, -87.0);
                cassini.Forward(32.0, -86.0, out x, out y, out azi, out rk);
                cassini.Forward(32.0, -86.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in CassiniSoldner.Forward");
                cassini.Reverse(x, y, out lat, out lon, out azi, out rk);
                cassini.Reverse(x, y, out x1, out y1);
                if (x1 != lat || y1 != lon)
                    throw new Exception("Error in CassiniSoldner.Reverse");
                Gnomonic gnomonic = new Gnomonic(m_geodesic);
                gnomonic = new Gnomonic();
                gnomonic.Forward(32.0, -86.0, 31.0, -87.0, out x, out y, out azi, out rk);
                gnomonic.Forward(32.0, -86.0, 31.0, -87.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in Gnomonic.Forward");
                gnomonic.Reverse(32.0, -86.0, x, y, out lat, out lon, out azi, out rk);
                gnomonic.Reverse(32.0, -86.0, x, y, out x1, out y1);
                if (x1 != lat || y1 != lon)
                    throw new Exception("Error in Gnomonic.Reverse");
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
