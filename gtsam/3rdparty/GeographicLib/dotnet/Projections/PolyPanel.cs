/**
 * \file NETGeographicLib\PolyPanel.cs
 * \brief NETGeographicLib.PolygonArea example
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
    public partial class PolyPanel : UserControl
    {
        PolygonArea m_poly = null;
        public PolyPanel()
        {
            InitializeComponent();
            m_poly = new PolygonArea(false);
            m_majorRadiusTextBox.Text = m_poly.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_poly.Flattening.ToString();
            m_lengthTextBox.Text = m_areaTextBox.Text = m_numPointsTextBox.Text = "0";
            m_currLatTextBox.Text = m_currLonTextBox.Text = "";
        }

        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                double a = Double.Parse(m_majorRadiusTextBox.Text);
                double f = Double.Parse(m_flatteningTextBox.Text);
                m_poly = new PolygonArea(new Geodesic(a, f), m_polylineCheckBox.Checked);
                m_lengthTextBox.Text = m_areaTextBox.Text = m_numPointsTextBox.Text = "0";
                m_currLatTextBox.Text = m_currLonTextBox.Text = "";
                m_edgeButton.Enabled = false;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnAddPoint(object sender, EventArgs e)
        {
            try
            {
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                double length, area;

                m_poly.AddPoint(lat, lon);
                m_edgeButton.Enabled = true;
                m_currLatTextBox.Text = m_latitudeTextBox.Text;
                m_currLonTextBox.Text = m_longitudeTextBox.Text;
                m_numPointsTextBox.Text = m_poly.Compute(false, false, out length, out area).ToString();
                m_lengthTextBox.Text = length.ToString();
                if ( !m_polylineCheckBox.Checked )
                    m_areaTextBox.Text = area.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnAddEdge(object sender, EventArgs e)
        {
            try
            {
                double azi = Double.Parse(m_azimuthTextBox.Text);
                double dist = Double.Parse(m_distanceTextBox.Text);

                m_poly.AddEdge(azi, dist);
                double lat, lon, length, area;
                m_poly.CurrentPoint(out lat, out lon);
                m_currLatTextBox.Text = lat.ToString();
                m_currLonTextBox.Text = lon.ToString();
                m_numPointsTextBox.Text = m_poly.Compute(false, false, out length, out area).ToString();
                m_lengthTextBox.Text = length.ToString();
                if (!m_polylineCheckBox.Checked)
                    m_areaTextBox.Text = area.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnClear(object sender, EventArgs e)
        {
            m_poly.Clear();
            m_edgeButton.Enabled = false;
            m_lengthTextBox.Text = m_areaTextBox.Text = m_numPointsTextBox.Text = "0";
            m_currLatTextBox.Text = m_currLonTextBox.Text = "";
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                double lat, lon, perim, area;
                PolygonArea pa = new PolygonArea(new Geodesic(50000.0, 0.001), true);
                pa = new PolygonArea(false);
                pa.AddPoint(32.0, -86.0);
                pa.AddEdge(20.0, 10000.0);
                pa.AddEdge(-45.0, 10000.0);
                pa.CurrentPoint(out lat, out lon);
                pa.Compute(false, false, out perim, out area);
                pa.TestEdge(-70.0, 5000.0, false, false, out perim, out area);
                pa.TestPoint(31.0, -86.5, false, false, out perim, out area);

                PolygonAreaExact p2 = new PolygonAreaExact(new GeodesicExact(), false);
                p2.AddPoint(32.0, -86.0);
                p2.AddEdge(20.0, 10000.0);
                p2.AddEdge(-45.0, 10000.0);
                p2.CurrentPoint(out lat, out lon);
                p2.Compute(false, false, out perim, out area);
                p2.TestEdge(-70.0, 5000.0, false, false, out perim, out area);
                p2.TestPoint(31.0, -86.5, false, false, out perim, out area);

                PolygonAreaRhumb p3 = new PolygonAreaRhumb(Rhumb.WGS84(), false);
                p3.AddPoint(32.0, -86.0);
                p3.AddEdge(20.0, 10000.0);
                p3.AddEdge(-45.0, 10000.0);
                p3.CurrentPoint(out lat, out lon);
                p3.Compute(false, false, out perim, out area);
                p3.TestEdge(-70.0, 5000.0, false, false, out perim, out area);
                p3.TestPoint(31.0, -86.5, false, false, out perim, out area);

                MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
