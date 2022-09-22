using System.Threading;

using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

namespace tshirt_cannon_robot
{
    public class Program
    {
        /* Motor controllers */
        private static TalonSRX frontRightMotor = new TalonSRX(3);
        private static TalonSRX backRightMotor = new TalonSRX(4);
        private static TalonSRX frontLeftMotor = new TalonSRX(1);
        private static TalonSRX backLeftMotor = new TalonSRX(2);

        /* Game controllers */
        private static GameController gameController = new GameController(new UsbHostDevice(0));

        public static void Main()
        {
            ConfigMotors();

            if (gameController.GetButton(5))

            while (true)
            {
                if (gameController.GetConnectionStatus() == UsbDeviceConnection.Connected)
                {
                    ArcadeDrive();

                    CTRE.Phoenix.Watchdog.Feed();
                    Thread.Sleep(10);
                }
            }
        }

        private static void ArcadeDrive()
        {
            float yAxis = gameController.GetAxis(1);
            float zAxis = gameController.GetAxis(0);

            frontRightMotor.Set(ControlMode.PercentOutput, yAxis);
            backRightMotor.Set(ControlMode.PercentOutput, yAxis);
            frontLeftMotor.Set(ControlMode.PercentOutput, yAxis);
            backLeftMotor.Set(ControlMode.PercentOutput, yAxis);
        }

        private static void ConfigMotors()
        {
            frontRightMotor.ConfigFactoryDefault();
            backRightMotor.ConfigFactoryDefault();
            frontLeftMotor.ConfigFactoryDefault();
            backLeftMotor.ConfigFactoryDefault();
        }
    }
}
