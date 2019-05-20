/**
 * \file NETGeographicLib\SphericalHarmonicsPanel.cs
 * \brief NETGeographicLib Spherical Harmonics example
 *
 * NETGeographicLib.CircularEngine,
 * NETGeographicLib.SphericalHarmonic,
 * NETGeographicLib.SphericalHarmonic1, and
 * NETGeographicLib.SphericalHarmonic2 example.
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
    public partial class SphericalHarmonicsPanel : UserControl
    {
        int N = 3, N1 = 2, N2 = 1;                     // The maxium degrees
        double[] C = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1}; // cosine coefficients
        double[] S = {6, 5, 4, 3, 2, 1}; // sine coefficients
        double[] C1 = {1, 2, 3, 4, 5, 6};
        double[] S1 = {3, 2, 1};
        double[] C2 = {1, 2, 3};
        double[] S2 = {1};
        double a = 1;

        SphericalHarmonic m_sh0 = null;
        SphericalHarmonic1 m_sh1 = null;
        SphericalHarmonic2 m_sh2 = null;

        public SphericalHarmonicsPanel()
        {
            InitializeComponent();
            try
            {
                m_sh0 = new SphericalHarmonic(C, S, N, a, SphericalHarmonic.Normalization.SCHMIDT);
                m_sh1 = new SphericalHarmonic1(C, S, N, C1, S1, N1, a, SphericalHarmonic1.Normalization.FULL);
                m_sh2 = new SphericalHarmonic2(C, S, N, C1, S1, N1, C2, S2, N2, a, SphericalHarmonic2.Normalization.FULL);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            m_classComboBox.SelectedIndex = 0;
        }

        private void OnClass(object sender, EventArgs e)
        {
            switch (m_classComboBox.SelectedIndex)
            {
                case 0:
                    m_tau1TextBox.ReadOnly = m_tau2TextBox.ReadOnly = true;
                    break;
                case 1:
                    m_tau1TextBox.ReadOnly = false;
                    m_tau2TextBox.ReadOnly = true;
                    break;
                case 2:
                    m_tau1TextBox.ReadOnly = m_tau2TextBox.ReadOnly = false;
                    break;
            }
        }

        private void OnCompute(object sender, EventArgs e)
        {
            try
            {
                double sum = 0.0, gradx = 0.0, grady = 0.0, gradz = 0.0;
                double x = Double.Parse(m_xTextBox.Text);
                double y = Double.Parse(m_yTextBox.Text);
                double z = Double.Parse(m_zTextBox.Text);
                switch (m_classComboBox.SelectedIndex)
                {
                    case 0:
                        sum = m_sh0.HarmonicSum(x, y, z, out gradx, out grady, out gradz);
                        break;
                    case 1:
                        double tau1 = Double.Parse(m_tau1TextBox.Text);
                        sum = m_sh1.HarmonicSum(tau1, x, y, z, out gradx, out grady, out gradz);
                        break;
                    case 2:
                        tau1 = Double.Parse(m_tau1TextBox.Text);
                        double tau2 = Double.Parse(m_tau2TextBox.Text);
                        sum = m_sh2.HarmonicSum(tau1, tau2, x, y, z, out gradx, out grady, out gradz);
                        break;
                }
                m_sumTextBox.Text = sum.ToString();
                m_gradXTextBox.Text = gradx.ToString();
                m_gradYTextBox.Text = grady.ToString();
                m_gradZTextBox.Text = gradz.ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnCircularEngine(object sender, EventArgs e)
        {
            try
            {
                CircularEngine ce = null;
                double p = Double.Parse(m_circleRadiusTextBox.Text);
                double z = Double.Parse(m_circleHeightTextBox.Text);
                double longitude = Double.Parse(m_longitudeTextBox.Text);

                switch (m_classComboBox.SelectedIndex)
                {
                    case 0:
                        ce = m_sh0.Circle(p, z, true);
                        break;
                    case 1:
                        double tau1 = Double.Parse(m_tau1TextBox.Text);
                        ce = m_sh1.Circle(tau1, p, z, true);
                        break;
                    case 2:
                        tau1 = Double.Parse(m_tau1TextBox.Text);
                        double tau2 = Double.Parse(m_tau2TextBox.Text);
                        ce = m_sh2.Circle(tau1, tau2, p, z, true);
                        break;
                }
                double gradx, grady, gradz;
                m_sumTextBox.Text = ce.LongitudeSum(longitude, out gradx, out grady, out gradz).ToString();
                m_gradXTextBox.Text = gradx.ToString();
                m_gradYTextBox.Text = grady.ToString();
                m_gradZTextBox.Text = gradz.ToString();
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
                const double DEG_TO_RAD = 3.1415926535897932384626433832795 / 180.0;
                double gradx, grady, gradz;
                SphericalHarmonic s0 = new SphericalHarmonic(C, S, N, N - 1, 0, a, SphericalHarmonic.Normalization.SCHMIDT);
                s0 = new SphericalHarmonic(C, S, N, a, SphericalHarmonic.Normalization.SCHMIDT);
                double sum = s0.HarmonicSum(1.0, 2.0, 3.0);
                double test = s0.HarmonicSum(1.0, 2.0, 3.0, out gradx, out grady, out grady);
                if (sum != test)
                    throw new Exception("Error in SphericalHarmonic.HarmonicSum");
                SphericalCoefficients sc = s0.Coefficients();
                CircularEngine ce = s0.Circle(1.0, 0.5, true);
                sum = ce.LongitudeSum(60.0);
                test = ce.LongitudeSum(Math.Cos(60.0 * DEG_TO_RAD), Math.Sin(60.0 * DEG_TO_RAD));
                if ( sum != test )
                    throw new Exception("Error in CircularEngine.LongitudeSum 1");
                test = ce.LongitudeSum(60.0, out gradx, out grady, out gradz);
                if ( sum != test )
                    throw new Exception("Error in CircularEngine.LongitudeSum 2");
                ce.LongitudeSum(Math.Cos(60.0 * DEG_TO_RAD), Math.Sin(60.0 * DEG_TO_RAD), out gradx, out grady, out gradz);
                if (sum != test)
                    throw new Exception("Error in CircularEngine.LongitudeSum 3");
                SphericalHarmonic1 s1 = new SphericalHarmonic1(C, S, N, N - 1, 1, C1, S1, N1, N1 - 1, 0, a, SphericalHarmonic1.Normalization.SCHMIDT);
                s1 = new SphericalHarmonic1(C, S, N, C1, S1, N1, a, SphericalHarmonic1.Normalization.SCHMIDT);
                sum = s1.HarmonicSum(0.95, 1.0, 2.0, 3.0);
                test = s1.HarmonicSum(0.95, 1.0, 2.0, 3.0, out gradx, out grady, out gradz);
                if (sum != test)
                    throw new Exception("Error in SphericalHarmonic1.HarmonicSum 3");
                ce = s1.Circle(0.95, 1.0, 0.5, true);
                sc = s1.Coefficients();
                sc = s1.Coefficients1();
                SphericalHarmonic2 s2 = new SphericalHarmonic2(C, S, N, N - 1, 2, C1, S1, N1, N1 - 1, 1,
                    C2, S2, N2, N2 - 1, 0, a, SphericalHarmonic2.Normalization.SCHMIDT);
                s2 = new SphericalHarmonic2(C, S, N, C1, S1, N1, C2, S2, N2, a, SphericalHarmonic2.Normalization.SCHMIDT);
                sum = s2.HarmonicSum(0.95, 0.8, 1.0, 2.0, 3.0);
                test = s2.HarmonicSum(0.95, 0.8, 1.0, 2.0, 3.0, out gradx, out grady, out gradz);
                if (sum != test)
                    throw new Exception("Error in SphericalHarmonic2.HarmonicSum 3");
                ce = s2.Circle(0.95, 0.8, 1.0, 0.5, true);
                sc = s2.Coefficients();
                sc = s2.Coefficients1();
                sc = s2.Coefficients2();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }
            MessageBox.Show("No errors found", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
