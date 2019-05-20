/**
 * \file NETGeographicLib\AccumPanel.cs
 * \brief NETGeographicLib.Accumulator example
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
    public partial class AccumPanel : UserControl
    {
        Accumulator m_accum = new Accumulator();

        public AccumPanel()
        {
            InitializeComponent();
            OnReset(null, null);

        }

        private void OnAdd(object sender, EventArgs e)
        {
            try
            {
                double a = Double.Parse(m_inputTextBox.Text);
                m_accum.Sum(a);
                m_accumTtextBox.Text = m_accum.Result().ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnMultiply(object sender, EventArgs e)
        {
            try
            {
                int a = Int32.Parse(m_inputTextBox.Text);
                m_accum.Multiply(a);
                m_accumTtextBox.Text = m_accum.Result().ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

        }

        private void OnReset(object sender, EventArgs e)
        {
            m_accum.Assign(0.0);
            m_accumTtextBox.Text = m_accum.Result().ToString();
        }

        private void OnTest(object sender, EventArgs e)
        {
            try
            {
                double test = Double.Parse(m_testTextBox.Text);
                m_equalsCheckBox.Checked = m_accum == test;
                m_lessThanCheckBox.Checked = m_accum < test;
                m_greaterTanCheckBox.Checked = m_accum > test;
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Exception", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
