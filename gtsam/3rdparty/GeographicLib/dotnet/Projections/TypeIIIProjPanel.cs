/**
 * \file NETGeographicLib\TypeIIIProjPanel.cs
 * \brief NETGeographicLib.projections example
 *
 * NETGeographicLib.UTMUPS,
 * NETGeographicLib.MGRS, and
 * NETGeographicLib.OSGB example.
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
    public partial class TypeIIIProjPanel : UserControl
    {
        public TypeIIIProjPanel()
        {
            InitializeComponent();
        }

        private void OnConvertLatLon(object sender, EventArgs e)
        {
            try
            {
                int zone;
                bool northp;
                double x, y;
                string str;
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                UTMUPS.Forward(lat, lon, out zone, out northp, out x, out y, (int)UTMUPS.ZoneSpec.STANDARD, true);
                m_utmPoleCheckBox.Checked = northp;
                m_utmXTextBox.Text = x.ToString();
                m_utmYTextBox.Text = y.ToString();
                m_utmZoneTextBox.Text = zone.ToString();
                MGRS.Forward(zone,northp,x,y,8,out str );
                m_mgrsTextBox.Text = str;
                OSGB.Forward(lat, lon, out x, out y);
                m_osgbXTextBox.Text = x.ToString();
                m_osgbYTextBox.Text = y.ToString();
                OSGB.GridReference(x, y, 8, out str);
                m_osgbTextBox.Text = str;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConvertUTMUPS(object sender, EventArgs e)
        {
            try
            {
                double lat, lon;
                string str;
                double x = Double.Parse(m_utmXTextBox.Text);
                double y = Double.Parse(m_utmYTextBox.Text);
                int zone = Int32.Parse(m_utmZoneTextBox.Text);
                UTMUPS.Reverse(zone, m_utmPoleCheckBox.Checked, x, y, out lat, out lon, true);
                m_latitudeTextBox.Text = lat.ToString();
                m_longitudeTextBox.Text = lon.ToString();
                MGRS.Forward(zone, m_utmPoleCheckBox.Checked, x, y, 8, out str);
                m_mgrsTextBox.Text = str;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConvertMGRS(object sender, EventArgs e)
        {
            int zone, prec;
            bool northp;
            double x, y, lat, lon;
            MGRS.Reverse(m_mgrsTextBox.Text, out zone, out northp, out x, out y, out prec, true);
            m_utmPoleCheckBox.Checked = northp;
            m_utmXTextBox.Text = x.ToString();
            m_utmYTextBox.Text = y.ToString();
            m_utmZoneTextBox.Text = zone.ToString();
            UTMUPS.Reverse(zone, northp, x, y, out lat, out lon, true);
            m_latitudeTextBox.Text = lat.ToString();
            m_longitudeTextBox.Text = lon.ToString();
        }

        private void OnConvertOSGB(object sender, EventArgs e)
        {
            // the latitude and longitude returned by OSGB is not WGS84 and should not
            // be used with UTMUPS or MGRS.
            double x = Double.Parse(m_osgbXTextBox.Text);
            double y = Double.Parse(m_osgbYTextBox.Text);
            double lat, lon;
            OSGB.Reverse(x, y, out lat, out lon);
            m_latitudeTextBox.Text = lat.ToString();
            m_longitudeTextBox.Text = lon.ToString();
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                string str;
                int prec, zone, zout;
                bool northp;
                double x, y, x1, y1, gamma, k, lat, lon;
                OSGB.Forward(52.0,-2.0, out x, out y, out gamma, out k);
                OSGB.Forward(52.0, -2.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in OSGB.Forward");
                OSGB.Reverse(x, y, out lat, out lon, out gamma, out k);
                OSGB.Reverse(x, y, out x1, out y1);
                if ( lat != x1 || lon != y1 )
                    throw new Exception("Error in OSGB.Reverse");
                OSGB.GridReference(x,y,8,out str);
                OSGB.GridReference(str, out x1, out y1, out prec, true);
                UTMUPS.StandardZone(32.0, -80.0, (int)UTMUPS.ZoneSpec.STANDARD);
                UTMUPS.UTMShift();
                UTMUPS.StandardZone(32.0, -86.0, (int)UTMUPS.ZoneSpec.STANDARD);
                UTMUPS.Forward(32.0, -86.0, out zone, out northp, out x, out y, out gamma, out k, (int)UTMUPS.ZoneSpec.STANDARD, true);
                UTMUPS.Forward(32.0, -86.0, out zone, out northp, out x1, out y1, (int)UTMUPS.ZoneSpec.STANDARD, true);
                if (x != x1 || y != y1)
                    throw new Exception("Error in UTMUPS.Forward");
                UTMUPS.Reverse(zone, northp, x, y, out lat, out lon, out gamma, out k, true);
                UTMUPS.Reverse(zone, northp, x, y, out x1, out y1, true);
                if (lat != x1 || lon != y1)
                    throw new Exception("Error in UTMUPS.Reverse");
                UTMUPS.Transfer(zone, northp, x, y, zone + 1, true, out x1, out y1, out zout);
                str = UTMUPS.EncodeZone(zone, northp, true);
                prec = UTMUPS.EncodeEPSG(zone, northp);
                UTMUPS.DecodeZone(str, out zone, out northp);
                UTMUPS.DecodeEPSG(prec, out zone, out northp);
                MGRS.Forward(zone, northp, x, y, 8, out str);
                MGRS.Forward(zone, northp, x, y, 32.0, 8, out str);
                MGRS.Reverse(str, out zone, out northp, out x, out y, out prec, true);

                MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
