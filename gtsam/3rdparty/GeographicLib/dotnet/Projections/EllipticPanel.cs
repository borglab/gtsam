/**
 * \file NETGeographicLib\EllipticPanel.cs
 * \brief NETGeographicLib.EllipticFunction example
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
    public partial class EllipticPanel : UserControl
    {
        EllipticFunction m_func = null;

        public EllipticPanel()
        {
            InitializeComponent();
            m_constructorComboBox.SelectedIndex = 0;
            m_k2TextBox.Text = "0.5";
            m_alpha2TextBox.Text = "0.5";
            OnSet(null, null);
        }

        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                double k2 = Double.Parse(m_k2TextBox.Text);
                double alpha2 = Double.Parse(m_alpha2TextBox.Text);
                if (m_constructorComboBox.SelectedIndex == 0)
                    m_func = new EllipticFunction(k2, alpha2);
                else
                {
                    double kp2 = Double.Parse(m_kp2TextBox.Text);
                    double alphap2 = Double.Parse(m_alphap2TextBox.Text);
                    m_func = new EllipticFunction(k2, alpha2, kp2, alphap2);
                }
                m_KtextBox.Text = m_func.K().ToString();
                m_EtextBox.Text = m_func.E().ToString();
                m_DtextBox.Text = m_func.D().ToString();
                m_KEtextBox.Text = m_func.KE().ToString();
                m_PItextBox.Text = m_func.Pi().ToString();
                m_GtextBox.Text = m_func.G().ToString();
                m_HtextBox.Text = m_func.H().ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConstructor(object sender, EventArgs e)
        {
            m_kp2TextBox.ReadOnly = m_alphap2TextBox.ReadOnly = m_constructorComboBox.SelectedIndex == 0;
        }

        private void OnComputePhi(object sender, EventArgs e)
        {
            try
            {
                double phi = Double.Parse(m_phiTextBox.Text);
                m_EphiTextBox.Text = m_func.E(phi).ToString();
                m_FphiTextBox.Text = m_func.F(phi).ToString();
                m_DphiTextBox.Text = m_func.D(phi).ToString();
                m_GphiTextBox.Text = m_func.G(phi).ToString();
                m_HphiTextBox.Text = m_func.H(phi).ToString();
                m_PiphiTextBox.Text = m_func.Pi(phi).ToString();
            }
            catch ( Exception xcpt )
            {
                MessageBox.Show( xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error );
            }
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                double phi = 0.8;
                EllipticFunction f = new EllipticFunction(0.3, 0.4, 0.7, 0.6);
                f.Reset(0.2, 0.3, 0.8, 0.7);
                f = new EllipticFunction(0.3, 0.4);
                f.Reset(0.2, 0.3);
                double cn, sn, dn;
                f.sncndn(0.3, out sn, out cn, out dn);
                f.Delta(sn, cn);
                f.D();
                f.D(phi);
                f.D(sn, cn, dn);
                f.Pi();
                f.Pi(phi);
                f.Pi(sn, cn, dn);
                f.KE();
                f.K();
                f.H();
                f.H(phi);
                f.H(sn, cn, dn);
                f.G();
                f.G(phi);
                f.G(sn, cn, dn);
                f.F(phi);
                f.F(sn, cn, dn);
                f.Einv(0.75);
                f.Ed(60.0);
                f.E();
                f.E(phi);
                f.E(sn, cn, dn);
                double tau = 3.1415927 / 10.0;
                f.deltaEinv(Math.Sin(tau), Math.Cos(tau));
                f.deltaD(sn, cn, dn);
                f.deltaE(sn, cn, dn);
                f.deltaF(sn, cn, dn);
                f.deltaG(sn, cn, dn);
                f.deltaH(sn, cn, dn);
                f.deltaPi(sn, cn, dn);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
