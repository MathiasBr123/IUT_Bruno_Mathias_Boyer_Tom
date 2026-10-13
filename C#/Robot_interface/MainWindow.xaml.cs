using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Robot_interface
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        private void TextBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void TextBox_TextChanged_1(object sender, TextChangedEventArgs e)
        {

        }

        private void bouttonEnvoyer_Click(object sender, RoutedEventArgs e)
        {
            String message = TextBoxEmission.Text;
            int firstPrint = 0;

            if (firstPrint == 0)
            {
                TextBoxReception.AppendText("Recu : " + message +"\n");
                firstPrint = 0;
            }else if (firstPrint == 1)
            {
                TextBoxReception.AppendText("\nRecu : " + message);
            }
            TextBoxEmission.Clear();
        }
    }
}