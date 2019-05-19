/**
 * \file NETGeographicLib\AlbersPanel.cs
 * \brief Example of various projections.
 *
 * NETGeographicLib.AlbersEqualArea,
 * NETGeographicLib.LambertConformalConic,
 * NETGeographicLib.TransverseMercator,
 * and NETGeographicLib.TransverseMercatorExact example
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
    public partial class AlbersPanel : UserControl
    {
        enum ProjectionTypes
        {
            AlbersEqualArea = 0,
            LambertConformalConic = 1,
            TransverseMercator = 2,
            TransverseMercatorExact = 3
        }
        ProjectionTypes m_projection;
        AlbersEqualArea m_albers = null;
        LambertConformalConic m_lambert = null;
        TransverseMercator m_trans = null;
        TransverseMercatorExact m_transExact = null;

        public AlbersPanel()
        {
            InitializeComponent();
            m_projectionComboBox.SelectedIndex = 0; // this calls OnProjection and sets it to an AlbersEqualArea
            m_constructorComboBox.SelectedIndex = 3; // this calls OnConstructorChanged
            m_majorRadiusTextBox.Text = m_albers.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_albers.Flattening.ToString();
            m_centralScaleTextBox.Text = m_albers.CentralScale.ToString();
            m_originLatitudeTextBox.Text = m_albers.OriginLatitude.ToString();
            m_functionComboBox.SelectedIndex = 0; // this calls OnFunction
        }

        private void OnConstructorChanged(object sender, EventArgs e)
        {
            try
            {
                if (m_projectionComboBox.SelectedIndex > 1)
                {
                    m_originLatitudeTextBox.ReadOnly = true;
                    m_originLatitudeTextBox.Text = "N/A";
                    if (m_constructorComboBox.SelectedIndex == 0)
                    {
                        m_convertButton.Enabled = true;
                        m_setButton.Enabled = false;
                        m_majorRadiusTextBox.ReadOnly = true;
                        m_flatteningTextBox.ReadOnly = true;
                        m_scaleLabel.Hide();
                        m_KTextBox.Hide();
                        m_stdLatLabel.Hide();
                        m_stdLat1TextBox.Hide();
                        m_stdLat2Label.Hide();
                        m_stdLat2TextBox.Hide();
                        m_sinLat2Label.Hide();
                        m_sinLat2TextBox.Hide();
                        m_cosLat2Label.Hide();
                        m_cosLat2TextBox.Hide();
                        if (m_projection == ProjectionTypes.TransverseMercator)
                        {
                            m_trans = new TransverseMercator();
                            m_centralScaleTextBox.Text = m_trans.CentralScale.ToString();
                        }
                        else
                        {
                            m_transExact = new TransverseMercatorExact();
                            m_centralScaleTextBox.Text = m_transExact.CentralScale.ToString();
                        }
                    }
                    else
                    {
                        m_convertButton.Enabled = false;
                        m_setButton.Enabled = true;
                        m_majorRadiusTextBox.ReadOnly = false;
                        m_flatteningTextBox.ReadOnly = false;
                        m_scaleLabel.Show();
                        m_KTextBox.Show();
                        m_stdLatLabel.Hide();
                        m_stdLat1TextBox.Hide();
                        m_stdLat2Label.Hide();
                        m_stdLat2TextBox.Hide();
                        m_sinLat2Label.Hide();
                        m_sinLat2TextBox.Hide();
                        m_cosLat2Label.Hide();
                        m_cosLat2TextBox.Hide();
                    }
                }
                else
                {
                    m_originLatitudeTextBox.ReadOnly = false;
                    switch (m_constructorComboBox.SelectedIndex)
                    {
                        case 0:
                            m_convertButton.Enabled = false;
                            m_setButton.Enabled = true;
                            m_majorRadiusTextBox.ReadOnly = false;
                            m_flatteningTextBox.ReadOnly = false;
                            m_scaleLabel.Show();
                            m_KTextBox.Show();
                            m_stdLatLabel.Show();
                            m_stdLatLabel.Text = "Standard Latitude (degrees)";
                            m_stdLat1TextBox.Show();
                            m_stdLat2Label.Hide();
                            m_stdLat2TextBox.Hide();
                            m_sinLat2Label.Hide();
                            m_sinLat2TextBox.Hide();
                            m_cosLat2Label.Hide();
                            m_cosLat2TextBox.Hide();
                            break;
                        case 1:
                            m_convertButton.Enabled = false;
                            m_setButton.Enabled = true;
                            m_majorRadiusTextBox.ReadOnly = false;
                            m_flatteningTextBox.ReadOnly = false;
                            m_scaleLabel.Show();
                            m_KTextBox.Show();
                            m_stdLatLabel.Show();
                            m_stdLatLabel.Text = "Standard Latitude 1 (degrees)";
                            m_stdLat1TextBox.Show();
                            m_stdLat2Label.Text = "Standard Latitude 2 (degrees)";
                            m_stdLat2Label.Show();
                            m_stdLat2TextBox.Show();
                            m_sinLat2Label.Hide();
                            m_sinLat2TextBox.Hide();
                            m_cosLat2Label.Hide();
                            m_cosLat2TextBox.Hide();
                            break;
                        case 2:
                            m_convertButton.Enabled = false;
                            m_setButton.Enabled = true;
                            m_majorRadiusTextBox.ReadOnly = false;
                            m_flatteningTextBox.ReadOnly = false;
                            m_scaleLabel.Show();
                            m_KTextBox.Show();
                            m_stdLatLabel.Show();
                            m_stdLatLabel.Text = "Sin(Lat1)";
                            m_stdLat1TextBox.Show();
                            m_stdLat2Label.Text = "Cos(Lat1)";
                            m_stdLat2Label.Show();
                            m_stdLat2TextBox.Show();
                            m_sinLat2Label.Show();
                            m_sinLat2TextBox.Show();
                            m_cosLat2Label.Show();
                            m_cosLat2TextBox.Show();
                            break;
                        default:
                            m_convertButton.Enabled = true;
                            m_setButton.Enabled = false;
                            m_majorRadiusTextBox.ReadOnly = true;
                            m_flatteningTextBox.ReadOnly = true;
                            m_scaleLabel.Hide();
                            m_KTextBox.Hide();
                            m_stdLatLabel.Hide();
                            m_stdLat1TextBox.Hide();
                            m_stdLat2Label.Hide();
                            m_stdLat2TextBox.Hide();
                            m_sinLat2Label.Hide();
                            m_sinLat2TextBox.Hide();
                            m_cosLat2Label.Hide();
                            m_cosLat2TextBox.Hide();
                            break;
                    }

                    if (m_projection == ProjectionTypes.AlbersEqualArea)
                        AlbersConstructorChanged();
                }
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void AlbersConstructorChanged()
        {
            switch (m_constructorComboBox.SelectedIndex)
            {
                case 3:
                    m_albers = new AlbersEqualArea(AlbersEqualArea.StandardTypes.CylindricalEqualArea);
                    break;
                case 4:
                    m_albers = new AlbersEqualArea(AlbersEqualArea.StandardTypes.AzimuthalEqualAreaNorth);
                    break;
                case 5:
                    m_albers = new AlbersEqualArea(AlbersEqualArea.StandardTypes.AzimuthalEqualAreaSouth);
                    break;
                default:
                    break;
            }

            if (m_constructorComboBox.SelectedIndex > 2)
            {
                m_majorRadiusTextBox.Text = m_albers.MajorRadius.ToString();
                m_flatteningTextBox.Text = m_albers.Flattening.ToString();
                m_centralScaleTextBox.Text = m_albers.CentralScale.ToString();
                m_originLatitudeTextBox.Text = m_albers.OriginLatitude.ToString();
            }
        }

        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                switch (m_projection)
                {
                    case ProjectionTypes.AlbersEqualArea:
                        SetAlbers();
                        break;
                    case ProjectionTypes.LambertConformalConic:
                        SetLambert();
                        break;
                    case ProjectionTypes.TransverseMercator:
                        SetTransverse();
                        break;
                    case ProjectionTypes.TransverseMercatorExact:
                        SetTransverseExact();
                        break;
                }
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            m_convertButton.Enabled = true;
        }

        private void SetAlbers()
        {
            double s2, s3, s4;
            double a = Double.Parse(m_majorRadiusTextBox.Text);
            double f = Double.Parse(m_flatteningTextBox.Text);
            double k = Double.Parse(m_KTextBox.Text);
            double s1 = Double.Parse(m_stdLat1TextBox.Text);
            switch (m_constructorComboBox.SelectedIndex)
            {
                case 0:
                    m_albers = new AlbersEqualArea(a, f, s1, k);
                    break;
                case 1:
                    s2 = Double.Parse(m_stdLat2TextBox.Text);
                    m_albers = new AlbersEqualArea(a, f, s1, s2, k);
                    break;
                case 2:
                    s2 = Double.Parse(m_stdLat2TextBox.Text);
                    s3 = Double.Parse(m_sinLat2TextBox.Text);
                    s4 = Double.Parse(m_cosLat2TextBox.Text);
                    m_albers = new AlbersEqualArea(a, f, s1, s2, s3, s4, k);
                    break;
                default:
                    break;
            }
            m_centralScaleTextBox.Text = m_albers.CentralScale.ToString();
            m_originLatitudeTextBox.Text = m_albers.OriginLatitude.ToString();
        }

        private void SetLambert()
        {
            double s2, s3, s4;
            double a = Double.Parse(m_majorRadiusTextBox.Text);
            double f = Double.Parse(m_flatteningTextBox.Text);
            double k = Double.Parse(m_KTextBox.Text);
            double s1 = Double.Parse(m_stdLat1TextBox.Text);
            switch (m_constructorComboBox.SelectedIndex)
            {
                case 0:
                    m_lambert = new LambertConformalConic(a, f, s1, k);
                    break;
                case 1:
                    s2 = Double.Parse(m_stdLat2TextBox.Text);
                    m_lambert = new LambertConformalConic(a, f, s1, s2, k);
                    break;
                case 2:
                    s2 = Double.Parse(m_stdLat2TextBox.Text);
                    s3 = Double.Parse(m_sinLat2TextBox.Text);
                    s4 = Double.Parse(m_cosLat2TextBox.Text);
                    m_lambert = new LambertConformalConic(a, f, s1, s2, s3, s4, k);
                    break;
                default:
                    break;
            }
            m_centralScaleTextBox.Text = m_lambert.CentralScale.ToString();
            m_originLatitudeTextBox.Text = m_lambert.OriginLatitude.ToString();
        }

        private void SetTransverse()
        {
            switch (m_constructorComboBox.SelectedIndex)
            {
                case 1:
                    double a = Double.Parse(m_majorRadiusTextBox.Text);
                    double f = Double.Parse(m_flatteningTextBox.Text);
                    double k = Double.Parse(m_KTextBox.Text);
                    m_trans = new TransverseMercator(a, f, k);
                    break;
                default:
                    break;
            }
            m_centralScaleTextBox.Text = m_trans.CentralScale.ToString();
        }

        private void SetTransverseExact()
        {
            switch (m_constructorComboBox.SelectedIndex)
            {
                case 1:
                    double a = Double.Parse(m_majorRadiusTextBox.Text);
                    double f = Double.Parse(m_flatteningTextBox.Text);
                    double k = Double.Parse(m_KTextBox.Text);
                    m_transExact = new TransverseMercatorExact(a, f, k, false);
                    break;
                default:
                    break;
            }
            m_centralScaleTextBox.Text = m_transExact.CentralScale.ToString();
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
                switch (m_projection)
                {
                    case ProjectionTypes.AlbersEqualArea:
                        ConvertAlbers();
                        break;
                    case ProjectionTypes.LambertConformalConic:
                        ConvertLambert();
                        break;
                    case ProjectionTypes.TransverseMercator:
                        ConvertTransverse();
                        break;
                    case ProjectionTypes.TransverseMercatorExact:
                        ConvertTransverseExact();
                        break;
                }
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void ConvertAlbers()
        {
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, gamma = 0.0, k = 0.0;
            double lon0 = Double.Parse(m_lon0TextBox.Text);
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_albers.Forward(lon0, lat, lon, out x, out y, out gamma, out k);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_albers.Reverse(lon0, x, y, out lat, out lon, out gamma, out k);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_gammaTextBox.Text = gamma.ToString();
            m_azimuthalScaleTextBox.Text = k.ToString();
        }

        private void ConvertLambert()
        {
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, gamma = 0.0, k = 0.0;
            double lon0 = Double.Parse(m_lon0TextBox.Text);
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_lambert.Forward(lon0, lat, lon, out x, out y, out gamma, out k);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_lambert.Reverse(lon0, x, y, out lat, out lon, out gamma, out k);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_gammaTextBox.Text = gamma.ToString();
            m_azimuthalScaleTextBox.Text = k.ToString();
        }

        private void ConvertTransverse()
        {
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, gamma = 0.0, k = 0.0;
            double lon0 = Double.Parse(m_lon0TextBox.Text);
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_trans.Forward(lon0, lat, lon, out x, out y, out gamma, out k);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_trans.Reverse(lon0, x, y, out lat, out lon, out gamma, out k);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_gammaTextBox.Text = gamma.ToString();
            m_azimuthalScaleTextBox.Text = k.ToString();
        }

        private void ConvertTransverseExact()
        {
            double lat = 0.0, lon = 0.0, x = 0.0, y = 0.0, gamma = 0.0, k = 0.0;
            double lon0 = Double.Parse(m_lon0TextBox.Text);
            switch (m_functionComboBox.SelectedIndex)
            {
                case 0:
                    lat = Double.Parse(m_latitudeTextBox.Text);
                    lon = Double.Parse(m_longitudeTextBox.Text);
                    m_transExact.Forward(lon0, lat, lon, out x, out y, out gamma, out k);
                    m_xTextBox.Text = x.ToString();
                    m_yTextBox.Text = y.ToString();
                    break;
                case 1:
                    x = Double.Parse(m_xTextBox.Text);
                    y = Double.Parse(m_yTextBox.Text);
                    m_transExact.Reverse(lon0, x, y, out lat, out lon, out gamma, out k);
                    m_latitudeTextBox.Text = lat.ToString();
                    m_longitudeTextBox.Text = lon.ToString();
                    break;
            }
            m_gammaTextBox.Text = gamma.ToString();
            m_azimuthalScaleTextBox.Text = k.ToString();
        }

        private void OnProjection(object sender, EventArgs e)
        {
            m_projection = (ProjectionTypes)m_projectionComboBox.SelectedIndex;
            int save = m_constructorComboBox.SelectedIndex;
            m_constructorComboBox.Items.Clear();
            if (m_projectionComboBox.SelectedIndex > 1) // TransverseMercator or TransverseMercatorExact
            {
                m_constructorComboBox.Items.Add("Default");
                m_constructorComboBox.Items.Add("Constructor #1");
            }
            else
            {
                m_constructorComboBox.Items.Add("Constructor #1");
                m_constructorComboBox.Items.Add("Constructor #2");
                m_constructorComboBox.Items.Add("Constructor #3");
                if (m_projection == ProjectionTypes.AlbersEqualArea)
                {
                    m_constructorComboBox.Items.Add("CylindricalEqualArea");
                    m_constructorComboBox.Items.Add("AzimuthalEqualAreaNorth");
                    m_constructorComboBox.Items.Add("AzimuthalEqualAreaSouth");
                }
            }
            // calls OnConstructor
            m_constructorComboBox.SelectedIndex = m_constructorComboBox.Items.Count > save ? save : 0;
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                const double DEG_TO_RAD = 3.1415926535897932384626433832795 / 180.0;
                AlbersEqualArea a = new AlbersEqualArea(AlbersEqualArea.StandardTypes.AzimuthalEqualAreaNorth);
                a = new AlbersEqualArea(AlbersEqualArea.StandardTypes.AzimuthalEqualAreaSouth);
                double radius = a.MajorRadius;
                double f = a.Flattening;
                a = new AlbersEqualArea(radius, f, 60.0, 1.0);
                a = new AlbersEqualArea(radius, f, 60.0, 70.0, 1.0);
                a = new AlbersEqualArea(radius, f, Math.Sin(88.0 * DEG_TO_RAD), Math.Cos(88.0 * DEG_TO_RAD),
                    Math.Sin(89.0*DEG_TO_RAD), Math.Cos(89.0*DEG_TO_RAD), 1.0);
                a = new AlbersEqualArea(AlbersEqualArea.StandardTypes.CylindricalEqualArea);
                double lon0 = 0.0, lat = 32.0, lon = -86.0, x, y, gamma, k, x1, y1;
                a.Forward(lon0, lat, lon, out x, out y, out gamma, out k);
                a.Forward(lon0, lat, lon, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in AlbersEqualArea.Forward");
                a.Reverse(lon0, x, y, out lat, out lon, out gamma, out k);
                a.Reverse(lon0, x, y, out x1, out y1);
                if (lat != x1 || lon != y1)
                    throw new Exception("Error in AlbersEqualArea.Reverse");
                LambertConformalConic b = new LambertConformalConic(radius, f, 60.0, 1.0);
                b = new LambertConformalConic(radius, f, 60.0, 65.0, 1.0);
                b = new LambertConformalConic(radius, f, Math.Sin(88.0 * DEG_TO_RAD), Math.Cos(88.0 * DEG_TO_RAD),
                    Math.Sin(89.0 * DEG_TO_RAD), Math.Cos(89.0 * DEG_TO_RAD), 1.0);
                b = new LambertConformalConic();
                b.SetScale(60.0, 1.0);
                b.Forward(-87.0, 32.0, -86.0, out x, out y, out gamma, out k);
                b.Forward(-87.0, 32.0, -86.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in LambertConformalConic.Forward");
                b.Reverse(-87.0, x, y, out lat, out lon, out gamma, out k);
                b.Reverse(-87.0, x, y, out x1, out y1, out gamma, out k);
                if (lat != x1 || lon != y1)
                    throw new Exception("Error in LambertConformalConic.Reverse");
                TransverseMercator c = new TransverseMercator(radius, f, 1.0);
                c = new TransverseMercator();
                c.Forward(-87.0, 32.0, -86.0, out x, out y, out gamma, out k);
                c.Forward(-87.0, 32.0, -86.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in TransverseMercator.Forward");
                c.Reverse(-87.0, x, y, out lat, out lon, out gamma, out k);
                c.Reverse(-87.0, x, y, out x1, out y1);
                if (lat != x1 || lon != y1)
                    throw new Exception("Error in TransverseMercator.Reverse");
                TransverseMercatorExact d = new TransverseMercatorExact(radius, f, 1.0, false);
                d = new TransverseMercatorExact();
                d.Forward(-87.0, 32.0, -86.0, out x, out y, out gamma, out k);
                d.Forward(-87.0, 32.0, -86.0, out x1, out y1);
                if (x != x1 || y != y1)
                    throw new Exception("Error in TransverseMercatorExact.Forward");
                d.Reverse(-87.0, x, y, out lat, out lon, out gamma, out k);
                d.Reverse(-87.0, x, y, out x1, out y1);
                if (lat != x1 || lon != y1)
                    throw new Exception("Error in TransverseMercatorExact.Reverse");
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
