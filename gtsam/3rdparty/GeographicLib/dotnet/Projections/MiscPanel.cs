/**
 * \file NETGeographicLib\MiscPanel.cs
 * \brief NETGeographicLib.DMS and NETGeographicLib.Geohash example
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
    public partial class MiscPanel : UserControl
    {
        public MiscPanel()
        {
            InitializeComponent();
            m_comboBox.SelectedIndex = 0;
        }

        private void OnConvertDMS(object sender, EventArgs e)
        {
            try
            {
                DMS.Flag ind;
                double lon = DMS.Decode(m_longitudeDMSTextBox.Text, out ind);
                m_LongitudeTextBox.Text = lon.ToString();
                double lat = DMS.Decode(m_latitudeDMSTextBox.Text, out ind);
                m_latitudeTextBox.Text = lat.ToString();
                string tmp = "";
                Geohash.Forward(lat, lon, 12, out tmp);
                m_geohashTextBox.Text = tmp;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConvert(object sender, EventArgs e)
        {
            try
            {
                double lon = Double.Parse(m_LongitudeTextBox.Text);
                double lat = Double.Parse(m_latitudeTextBox.Text);
                m_longitudeDMSTextBox.Text = DMS.Encode(lon, 5, DMS.Flag.LONGITUDE, 0);
                m_latitudeDMSTextBox.Text = DMS.Encode(lat, 5, DMS.Flag.LATITUDE, 0);
                string tmp = "";
                switch (m_comboBox.SelectedIndex)
                {
                    case 0: // Geohash
                        Geohash.Forward(lat, lon, 12, out tmp);
                        break;
                    case 1: // GARS
                        GARS.Forward(lat, lon, 2, out tmp);
                        break;
                    case 2: // Georef
                        Georef.Forward(lat, lon, 2, out tmp);
                        break;
                }
                m_geohashTextBox.Text = tmp;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConvertGeohash(object sender, EventArgs e)
        {
            try
            {
                double lat = 0.0, lon = 0.0;
                int len;
                switch (m_comboBox.SelectedIndex)
                {
                    case 0: // Geohash
                        Geohash.Reverse(m_geohashTextBox.Text, out lat, out lon, out len, true);
                        break;
                    case 1: // GARS
                        GARS.Reverse(m_geohashTextBox.Text, out lat, out lon, out len, true);
                        break;
                    case 2: // Georef
                        Georef.Reverse(m_geohashTextBox.Text, out lat, out lon, out len, true);
                        break;
                }
                m_LongitudeTextBox.Text = lon.ToString();
                m_latitudeTextBox.Text = lat.ToString();
                m_longitudeDMSTextBox.Text = DMS.Encode(lon, 5, DMS.Flag.LONGITUDE, 0);
                m_latitudeDMSTextBox.Text = DMS.Encode(lat, 5, DMS.Flag.LATITUDE, 0);
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
                double lat, lon, d, m, s;
                DMS.Flag ind;
                int len;
                string tmp;
                DMS.Decode("34d22\'34.567\"", out ind);
                DMS.Decode(-86.0, 32.0, 34.214);
                DMS.DecodeAngle("-67.4532");
                DMS.DecodeAzimuth("85.3245W");
                DMS.DecodeLatLon("86d34\'24.5621\"", "21d56\'32.1234\"", out lat, out lon, false);
                DMS.Encode(-86.453214, out d, out m);
                DMS.Encode(-86.453214, out d, out m, out s);
                DMS.Encode(-86.453214, DMS.Component.SECOND, 12, DMS.Flag.LONGITUDE, 0);
                DMS.Encode(-86.453214, 12, DMS.Flag.LONGITUDE, 0);
                Geohash.DecimalPrecision(12);
                Geohash.Forward(31.23456, -86.43678, 12, out tmp);
                Geohash.GeohashLength(0.001);
                Geohash.GeohashLength(0.002, 0.003);
                Geohash.LatitudeResolution(12);
                Geohash.LongitudeResolution(12);
                Geohash.Reverse("djds54mrfc0g", out lat, out lon, out len, true);
                GARS.Forward(32.0, -86.0, 2, out tmp);
                GARS.Reverse("189LE37", out lat, out lon, out len, true);
                Georef.Forward(32.0, -86.0, 2, out tmp);
                Georef.Reverse("GJEC0000", out lat, out lon, out len, true);
                MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
