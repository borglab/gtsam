/**
 * \file NETGeographicLib\EllipsoidPanel.cs
 * \brief NETGeographicLib.Ellipsoid example
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
    public partial class EllipsoidPanel : UserControl
    {
        Ellipsoid m_ell = null;
        public EllipsoidPanel()
        {
            InitializeComponent();
            m_ell = new Ellipsoid();
            m_majorRadiusTextBox.Text = m_ell.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_ell.Flattening.ToString();

            m_minorRadiusTextBox.Text = m_ell.MinorRadius.ToString();
            m_quarterMeridianTextBox.Text = m_ell.QuarterMeridian.ToString();
            m_areaTextBox.Text = m_ell.Area.ToString();
            m_volumeTextBox.Text = m_ell.Volume.ToString();
            m_2ndFlatTextBox.Text = m_ell.SecondFlattening.ToString();
            m_3rdFlatTextBox.Text = m_ell.ThirdFlattening.ToString();
            m_ecc2TextBox.Text = m_ell.EccentricitySq.ToString();
            m_2ecc2TextBox.Text = m_ell.SecondEccentricitySq.ToString();
        }

        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                double a = Double.Parse(m_majorRadiusTextBox.Text);
                double f = Double.Parse(m_flatteningTextBox.Text);
                m_ell = new Ellipsoid(a, f);

                m_minorRadiusTextBox.Text = m_ell.MinorRadius.ToString();
                m_quarterMeridianTextBox.Text = m_ell.QuarterMeridian.ToString();
                m_areaTextBox.Text = m_ell.Area.ToString();
                m_volumeTextBox.Text = m_ell.Volume.ToString();
                m_2ndFlatTextBox.Text = m_ell.SecondFlattening.ToString();
                m_3rdFlatTextBox.Text = m_ell.ThirdFlattening.ToString();
                m_ecc2TextBox.Text = m_ell.EccentricitySq.ToString();
                m_2ecc2TextBox.Text = m_ell.SecondEccentricitySq.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnCalculateLatitudes(object sender, EventArgs e)
        {
            try
            {
                double phi = Double.Parse(m_phiTextBox.Text);
                m_parametericLatTextBox.Text = m_ell.ParametricLatitude(phi).ToString();
                m_geocentricLatTextBox.Text = m_ell.GeocentricLatitude(phi).ToString();
                m_rectifyingLatTextBox.Text = m_ell.RectifyingLatitude(phi).ToString();
                m_authalicLatTextBox.Text = m_ell.AuthalicLatitude(phi).ToString();
                m_conformalTextBox.Text = m_ell.ConformalLatitude(phi).ToString();
                m_isometricLatTextBox.Text = m_ell.IsometricLatitude(phi).ToString();
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
                Ellipsoid ee = new Ellipsoid(50000.0, .003);
                ee = new Ellipsoid();
                ee.AuthalicLatitude(30.0);
                ee.CircleHeight(30.0);
                ee.CircleRadius(30.0);
                ee.ConformalLatitude(30.0);
                ee.GeocentricLatitude(30.0);
                ee.InverseAuthalicLatitude(30.0);
                ee.InverseConformalLatitude(30.0);
                ee.InverseGeocentricLatitude(30.0);
                ee.InverseIsometricLatitude(30.0);
                ee.InverseParametricLatitude(30.0);
                ee.InverseRectifyingLatitude(30.0);
                ee.IsometricLatitude(30.0);
                ee.MeridianDistance(30.0);
                ee.MeridionalCurvatureRadius(30.0);
                ee.NormalCurvatureRadius(30.0, 60.0);
                ee.ParametricLatitude(30.0);
                ee.RectifyingLatitude(30.0);
                ee.TransverseCurvatureRadius(30.0);

                MessageBox.Show("no errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
