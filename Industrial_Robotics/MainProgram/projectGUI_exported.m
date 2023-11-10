classdef projectGUI_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                     matlab.ui.Figure
        RailLabel                    matlab.ui.control.Label
        RobotToControlSwitch         matlab.ui.control.Switch
        RobotToControlSwitchLabel    matlab.ui.control.Label
        VzEditFieldLabel_2           matlab.ui.control.Label
        VyEditFieldLabel_2           matlab.ui.control.Label
        VxEditFieldLabel_2           matlab.ui.control.Label
        VzEditField_2                matlab.ui.control.NumericEditField
        VyEditField_2                matlab.ui.control.NumericEditField
        VxEditField_2                matlab.ui.control.NumericEditField
        VzEditField                  matlab.ui.control.NumericEditField
        VzEditFieldLabel             matlab.ui.control.Label
        VyEditField                  matlab.ui.control.NumericEditField
        VyEditFieldLabel             matlab.ui.control.Label
        VxEditField                  matlab.ui.control.NumericEditField
        VxEditFieldLabel             matlab.ui.control.Label
        WxButton_2                   matlab.ui.control.Button
        WyButton_2                   matlab.ui.control.Button
        WyButton                     matlab.ui.control.Button
        WxButton                     matlab.ui.control.Button
        WzButton_2                   matlab.ui.control.Button
        WzButton                     matlab.ui.control.Button
        VxButton_2                   matlab.ui.control.Button
        VyButton_2                   matlab.ui.control.Button
        VyButton                     matlab.ui.control.Button
        VxButton                     matlab.ui.control.Button
        VzButton_2                   matlab.ui.control.Button
        VzButton                     matlab.ui.control.Button
        CartesianControlLabel        matlab.ui.control.Label
        ModeSwitch                   matlab.ui.control.Switch
        ModeSwitchLabel              matlab.ui.control.Label
        STARTButton                  matlab.ui.control.Button
        DisengageButton              matlab.ui.control.Button
        ResumeButton                 matlab.ui.control.Button
        eSTOPButton                  matlab.ui.control.Button
        q6Slider                     matlab.ui.control.Slider
        q5Slider                     matlab.ui.control.Slider
        q4Slider                     matlab.ui.control.Slider
        q3Slider                     matlab.ui.control.Slider
        q2Slider                     matlab.ui.control.Slider
        q1Slider                     matlab.ui.control.Slider
        RailSlider                   matlab.ui.control.Slider
        dZSpinner                    matlab.ui.control.Spinner
        dZSpinnerLabel               matlab.ui.control.Label
        dYSpinner                    matlab.ui.control.Spinner
        dYSpinnerLabel               matlab.ui.control.Label
        dXSpinner                    matlab.ui.control.Spinner
        dXSpinnerLabel               matlab.ui.control.Label
        RollSpinner                  matlab.ui.control.Spinner
        RollSpinnerLabel             matlab.ui.control.Label
        PitchSpinner                 matlab.ui.control.Spinner
        PitchSpinnerLabel            matlab.ui.control.Label
        YawSpinner                   matlab.ui.control.Spinner
        YawSpinnerLabel              matlab.ui.control.Label
        JointPositionControlLabel    matlab.ui.control.Label
        JointPostionControlCheckBox  matlab.ui.control.CheckBox
        EndEffectorStepCheckBox      matlab.ui.control.CheckBox
        TrajectoryGeneratorDropDown  matlab.ui.control.DropDown
        EndEffectorStepLabel         matlab.ui.control.Label
        ExecuteButton                matlab.ui.control.Button
        PlanButton                   matlab.ui.control.Button
        q7SliderLabel                matlab.ui.control.Label
        q6SliderLabel                matlab.ui.control.Label
        q5SliderLabel                matlab.ui.control.Label
        q4SliderLabel                matlab.ui.control.Label
        q3SliderLabel                matlab.ui.control.Label
        q2SliderLabel                matlab.ui.control.Label
    end

    
    properties (Access = private)

        tm5_control;
        ur3e_control;
        qMatrixTM5;
        qMatrixUR3e;
        disengaged = true;
    end
    
    methods (Access = private)

    end
    methods (Access = public)
        function setTM5_ctrl(app,tm5_ctrl)
            app.tm5_control = tm5_ctrl;
        end

        function setUR3e_ctrl(app,ur3e_ctrl)
            app.ur3e_control = ur3e_ctrl;
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            
        end

        % Button pushed function: PlanButton
        function PlanButtonPushed(app, event)
            if app.ModeSwitch.Value == "Manual"
               if app.EndEffectorStepCheckBox.Value == 1
                   dX = app.dXSpinner.Value;
                   dY = app.dYSpinner.Value;
                   dZ = app.dZSpinner.Value;
                   dYaw = app.YawSpinner.Value;
                   dPitch = app.PitchSpinner.Value;
                   dRoll = app.RollSpinner.Value;

                   transformation = transl(dX,dY,dZ) * rpy2tr(deg2rad(dYaw),deg2rad(dPitch),deg2rad(dRoll));

                   if app.RobotToControlSwitch.Value == "UR3e"
                        targetPose = app.ur3e_control.r.model.fkine(app.ur3e_control.r.model.getpos()) * SE3(transformation);
                   elseif app.RobotToControlSwitch.Value == "TM5"
                        targetPose = app.tm5_control.r.model.fkine(app.tm5_control.r.model.getpos()) * SE3(transformation);
                   end


                   
                   %Trap ikine
                   if app.TrajectoryGeneratorDropDown == 1
                       if app.RobotToControlSwitch.Value == "UR3e"
                           app.qMatrixUR3e = app.ur3e_control.trap_ikine(app.ur3e_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                       elseif app.RobotToControlSwitch.Value == "TM5"
                            app.qMatrixUR3e = app.tm5_control.trap_ikine(app.tm5_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                       end
                   %Trap ikcon
                   elseif app.TrajectoryGeneratorDropDown == 2
                       if app.RobotToControlSwitch.Value == "UR3e"
                           app.qMatrixUR3e = app.ur3e_control.trap_ikcon(app.ur3e_control.r.model.getpos(),0,targetPose,50);
                       elseif app.RobotToControlSwitch.Value == "TM5"
                           app.qMatrixTM5 = app.tm5_control.trap_ikcon(app.tm5_control.r.model.getpos(),0,targetPose,50);
                       end
                   %Quintic ikine
                   elseif app.TrajectoryGeneratorDropDown == 3
                       if app.RobotToControlSwitch.Value == "UR3e"
                            app.qMatrixUR3e = app.ur3e_control.quintic_ikine(app.ur3e_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                       elseif app.RobotToControlSwitch.Value == "TM5"
                            app.qMatrixTM5 = app.tm5_control.quintic_ikine(app.tm5_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                       end
                   %Quintic ikcon
                   elseif app.TrajectoryGeneratorDropDown == 4
                       if app.RobotToControlSwitch.Value == "UR3e"
                            app.qMatrixUR3e = app.ur3e_control.quintic_ikcon(app.ur3e_control.r.model.getpos(),0,targetPose,50);
                       elseif app.RobotToControlSwitch.Value == "TM5"
                            app.qMatrixTM5 = app.tm5_control.quintic_ikcon(app.tm5_control.r.model.getpos(),0,targetPose,50);
                       end
                   end
               elseif app.JointPostionControlCheckBox.Value == 1
                   if app.RobotToControlSwitch.Value == "UR3e"
                       %UR3e Joint Controlled          
                       q1 = app.q1Slider.Value;
                       q2 = app.q2Slider.Value;
                       q3 = app.q3Slider.Value;
                       q4 = app.q4Slider.Value;
                       q5 = app.q5Slider.Value;
                       q6 = app.q6Slider.Value;
                       jointState = [deg2rad(q1),deg2rad(q2),deg2rad(q3),deg2rad(q4),deg2rad(q5),deg2rad(q6)];
                       targetPose = app.ur3e_control.r.model.fkine(jointState);
                       if app.TrajectoryGeneratorDropDown.Value == "Trapezoidal ikine"
                           app.qMatrixUR3e = app.ur3e_control.trap_ikine(app.ur3e_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                       elseif app.TrajectoryGeneratorDropDown.Value == "Trapezoidal ikcon"
                           app.qMatrixUR3e = app.ur3e_control.trap_ikcon(app.ur3e_control.r.model.getpos(),0,targetPose,50);
                       elseif app.TrajectoryGeneratorDropDown.Value == "Quintic ikine"
                        app.qMatrixUR3e = app.ur3e_control.quintic_ikine(app.ur3e_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1],50);
                       elseif app.TrajectoryGeneratorDropDown.Value == "Quintic ikcon"
                           app.qMatrixUR3e = app.ur3e_control.quintic_ikcon(app.ur3e_control.r.model.getpos(),0,targetPose,50);
                       end
                   elseif app.RobotToControlSwitch.Value == "TM5"
                       q1 = app.RailSlider.Value;
                       q2 = app.q1Slider.Value;
                       q3 = app.q2Slider.Value;
                       q4 = app.q3Slider.Value;
                       q5 = app.q4Slider.Value;
                       q6 = app.q5Slider.Value;
                       q7 = app.q6Slider.Value;
                       jointState = [q1,deg2rad(q2),deg2rad(q3),deg2rad(q4),deg2rad(q5),deg2rad(q6),deg2rad(q7)];
                       targetPose = app.tm5_control.r.model.fkine(jointState);
                       if app.TrajectoryGeneratorDropDown.Value == "Trapezoidal ikine"
                           app.qMatrixTM5 = app.tm5_control.trap_ikine(app.tm5_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1 1],50);
                       elseif app.TrajectoryGeneratorDropDown.Value == "Trapezoidal ikcon"
                           app.qMatrixTM5 = app.tm5_control.trap_ikcon(app.tm5_control.r.model.getpos(),0,targetPose,50);
                       elseif app.TrajectoryGeneratorDropDown.Value == "Quintic ikine"
                            app.qMatrixTM5 = app.tm5_control.quintic_ikine(app.tm5_control.r.model.getpos(),0,targetPose,[1 1 1 1 1 1 1],50);
                       elseif app.TrajectoryGeneratorDropDown.Value == "Quintic ikcon"
                            app.qMatrixTM5 = app.tm5_control.quintic_ikcon(app.tm5_control.r.model.getpos(),0,targetPose,50);
                       end
                   end
               end
               %Plot the planned path
               if app.RobotToControlSwitch.Value == "UR3e"
                   app.ur3e_control.plotTrajectory(app.qMatrixUR3e)
               elseif app.RobotToControlSwitch.Value == "TM5"
                   app.tm5_control.plotTrajectory(app.qMatrixTM5)
               end
            end

        end

        % Button pushed function: ExecuteButton
        function ExecuteButtonPushed(app, event)
            if app.RobotToControlSwitch.Value == "UR3e"
                app.ur3e_control.executeTraj(app.qMatrixUR3e);
            elseif app.RobotToControlSwitch.Value == "TM5"
                app.tm5_control.executeTraj(app.qMatrixTM5);
            end
        end

        % Button pushed function: STARTButton
        function STARTButtonPushed(app, event)
            curtainTest;
        end

        % Button pushed function: ResumeButton
        function ResumeButtonPushed(app, event)
            if (app.disengaged)
                app.tm5_control.estop = false;
                app.ur3e_control.estop = false;
                app.ur3e_control.continueTraj();
            end
        end

        % Button pushed function: eSTOPButton
        function eSTOPButtonPushed(app, event)
            app.tm5_control.estop = true;
            app.ur3e_control.estop = true;
            app.disengaged = false;
        end

        % Button pushed function: VzButton
        function VzButtonPushed(app, event)          
            
        end

        % Button pushed function: VzButton_2
        function VzButton_2Pushed(app, event)
            
        end

        % Button pushed function: VxButton
        function VxButtonPushed(app, event)
            
        end

        % Button pushed function: VyButton
        function VyButtonPushed(app, event)
            
        end

        % Button pushed function: VyButton_2
        function VyButton_2Pushed(app, event)
            
        end

        % Button pushed function: VxButton_2
        function VxButton_2Pushed(app, event)
            
        end

        % Button pushed function: WzButton
        function WzButtonPushed(app, event)
            
        end

        % Button pushed function: WzButton_2
        function WzButton_2Pushed(app, event)
            
        end

        % Button pushed function: WxButton
        function WxButtonPushed(app, event)
            
        end

        % Button pushed function: WyButton
        function WyButtonPushed(app, event)
            
        end

        % Button pushed function: WyButton_2
        function WyButton_2Pushed(app, event)
            
        end

        % Button pushed function: WxButton_2
        function WxButton_2Pushed(app, event)
            
        end

        % Button pushed function: DisengageButton
        function DisengageButtonPushed(app, event)
            app.disengaged = true;
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 488 738];
            app.UIFigure.Name = 'MATLAB App';

            % Create q2SliderLabel
            app.q2SliderLabel = uilabel(app.UIFigure);
            app.q2SliderLabel.HorizontalAlignment = 'right';
            app.q2SliderLabel.Position = [7 624 25 22];
            app.q2SliderLabel.Text = 'q1';

            % Create q3SliderLabel
            app.q3SliderLabel = uilabel(app.UIFigure);
            app.q3SliderLabel.HorizontalAlignment = 'right';
            app.q3SliderLabel.Position = [7 583 25 22];
            app.q3SliderLabel.Text = 'q2';

            % Create q4SliderLabel
            app.q4SliderLabel = uilabel(app.UIFigure);
            app.q4SliderLabel.HorizontalAlignment = 'right';
            app.q4SliderLabel.Position = [7 535 25 22];
            app.q4SliderLabel.Text = 'q3';

            % Create q5SliderLabel
            app.q5SliderLabel = uilabel(app.UIFigure);
            app.q5SliderLabel.HorizontalAlignment = 'right';
            app.q5SliderLabel.Position = [7 494 25 22];
            app.q5SliderLabel.Text = 'q4';

            % Create q6SliderLabel
            app.q6SliderLabel = uilabel(app.UIFigure);
            app.q6SliderLabel.HorizontalAlignment = 'right';
            app.q6SliderLabel.Position = [7 446 25 22];
            app.q6SliderLabel.Text = 'q5';

            % Create q7SliderLabel
            app.q7SliderLabel = uilabel(app.UIFigure);
            app.q7SliderLabel.HorizontalAlignment = 'right';
            app.q7SliderLabel.Position = [7 403 25 22];
            app.q7SliderLabel.Text = 'q6';

            % Create PlanButton
            app.PlanButton = uibutton(app.UIFigure, 'push');
            app.PlanButton.ButtonPushedFcn = createCallbackFcn(app, @PlanButtonPushed, true);
            app.PlanButton.Position = [61 66 100 42];
            app.PlanButton.Text = 'Plan';

            % Create ExecuteButton
            app.ExecuteButton = uibutton(app.UIFigure, 'push');
            app.ExecuteButton.ButtonPushedFcn = createCallbackFcn(app, @ExecuteButtonPushed, true);
            app.ExecuteButton.Position = [61 15 100 43];
            app.ExecuteButton.Text = 'Execute';

            % Create EndEffectorStepLabel
            app.EndEffectorStepLabel = uilabel(app.UIFigure);
            app.EndEffectorStepLabel.Position = [65 337 109 22];
            app.EndEffectorStepLabel.Text = 'End-Effector Step';

            % Create TrajectoryGeneratorDropDown
            app.TrajectoryGeneratorDropDown = uidropdown(app.UIFigure);
            app.TrajectoryGeneratorDropDown.Items = {'Trapezoidal ikine', 'Trapezoidal ikcon', 'Quintic ikine', 'Quintic ikcon'};
            app.TrajectoryGeneratorDropDown.Position = [19 172 183 21];
            app.TrajectoryGeneratorDropDown.Value = 'Trapezoidal ikcon';

            % Create EndEffectorStepCheckBox
            app.EndEffectorStepCheckBox = uicheckbox(app.UIFigure);
            app.EndEffectorStepCheckBox.Text = 'End Effector Step';
            app.EndEffectorStepCheckBox.Position = [65 137 116 22];

            % Create JointPostionControlCheckBox
            app.JointPostionControlCheckBox = uicheckbox(app.UIFigure);
            app.JointPostionControlCheckBox.Text = 'Joint Postion Control';
            app.JointPostionControlCheckBox.Position = [65 116 133 22];

            % Create JointPositionControlLabel
            app.JointPositionControlLabel = uilabel(app.UIFigure);
            app.JointPositionControlLabel.Position = [76 698 118 24];
            app.JointPositionControlLabel.Text = 'Joint Position Control';

            % Create YawSpinnerLabel
            app.YawSpinnerLabel = uilabel(app.UIFigure);
            app.YawSpinnerLabel.HorizontalAlignment = 'right';
            app.YawSpinnerLabel.Position = [111 304 27 22];
            app.YawSpinnerLabel.Text = 'Yaw';

            % Create YawSpinner
            app.YawSpinner = uispinner(app.UIFigure);
            app.YawSpinner.Position = [146 304 60 22];

            % Create PitchSpinnerLabel
            app.PitchSpinnerLabel = uilabel(app.UIFigure);
            app.PitchSpinnerLabel.HorizontalAlignment = 'right';
            app.PitchSpinnerLabel.Position = [109 263 32 22];
            app.PitchSpinnerLabel.Text = 'Pitch';

            % Create PitchSpinner
            app.PitchSpinner = uispinner(app.UIFigure);
            app.PitchSpinner.Position = [146 263 60 22];

            % Create RollSpinnerLabel
            app.RollSpinnerLabel = uilabel(app.UIFigure);
            app.RollSpinnerLabel.HorizontalAlignment = 'right';
            app.RollSpinnerLabel.Position = [105 221 33 22];
            app.RollSpinnerLabel.Text = 'Roll';

            % Create RollSpinner
            app.RollSpinner = uispinner(app.UIFigure);
            app.RollSpinner.Position = [146 222 61 23];

            % Create dXSpinnerLabel
            app.dXSpinnerLabel = uilabel(app.UIFigure);
            app.dXSpinnerLabel.HorizontalAlignment = 'right';
            app.dXSpinnerLabel.Position = [13 304 25 22];
            app.dXSpinnerLabel.Text = 'dX';

            % Create dXSpinner
            app.dXSpinner = uispinner(app.UIFigure);
            app.dXSpinner.Position = [43 304 60 22];

            % Create dYSpinnerLabel
            app.dYSpinnerLabel = uilabel(app.UIFigure);
            app.dYSpinnerLabel.HorizontalAlignment = 'right';
            app.dYSpinnerLabel.Position = [12 263 25 22];
            app.dYSpinnerLabel.Text = 'dY';

            % Create dYSpinner
            app.dYSpinner = uispinner(app.UIFigure);
            app.dYSpinner.Position = [46 263 57 22];

            % Create dZSpinnerLabel
            app.dZSpinnerLabel = uilabel(app.UIFigure);
            app.dZSpinnerLabel.HorizontalAlignment = 'right';
            app.dZSpinnerLabel.Position = [5 221 33 22];
            app.dZSpinnerLabel.Text = 'dZ';

            % Create dZSpinner
            app.dZSpinner = uispinner(app.UIFigure);
            app.dZSpinner.Position = [45 222 58 23];

            % Create RailSlider
            app.RailSlider = uislider(app.UIFigure);
            app.RailSlider.Limits = [-0.8 -0.01];
            app.RailSlider.Position = [53 685 147 3];
            app.RailSlider.Value = -0.01;

            % Create q1Slider
            app.q1Slider = uislider(app.UIFigure);
            app.q1Slider.Limits = [-270 270];
            app.q1Slider.Position = [53 633 150 3];

            % Create q2Slider
            app.q2Slider = uislider(app.UIFigure);
            app.q2Slider.Limits = [-180 180];
            app.q2Slider.Position = [53 592 150 3];

            % Create q3Slider
            app.q3Slider = uislider(app.UIFigure);
            app.q3Slider.Limits = [-180 180];
            app.q3Slider.Position = [53 544 150 3];

            % Create q4Slider
            app.q4Slider = uislider(app.UIFigure);
            app.q4Slider.Limits = [-180 180];
            app.q4Slider.Position = [53 503 150 3];

            % Create q5Slider
            app.q5Slider = uislider(app.UIFigure);
            app.q5Slider.Limits = [-180 180];
            app.q5Slider.Position = [53 455 150 3];

            % Create q6Slider
            app.q6Slider = uislider(app.UIFigure);
            app.q6Slider.Limits = [-270 270];
            app.q6Slider.Position = [53 412 150 3];

            % Create eSTOPButton
            app.eSTOPButton = uibutton(app.UIFigure, 'push');
            app.eSTOPButton.ButtonPushedFcn = createCallbackFcn(app, @eSTOPButtonPushed, true);
            app.eSTOPButton.BackgroundColor = [1 0 0];
            app.eSTOPButton.FontColor = [1 1 0];
            app.eSTOPButton.Position = [365 107 98 62];
            app.eSTOPButton.Text = 'eSTOP';

            % Create ResumeButton
            app.ResumeButton = uibutton(app.UIFigure, 'push');
            app.ResumeButton.ButtonPushedFcn = createCallbackFcn(app, @ResumeButtonPushed, true);
            app.ResumeButton.Position = [374 22 82 62];
            app.ResumeButton.Text = 'Resume';

            % Create DisengageButton
            app.DisengageButton = uibutton(app.UIFigure, 'push');
            app.DisengageButton.ButtonPushedFcn = createCallbackFcn(app, @DisengageButtonPushed, true);
            app.DisengageButton.BackgroundColor = [0 0 1];
            app.DisengageButton.FontColor = [1 1 0];
            app.DisengageButton.Position = [244 107 98 62];
            app.DisengageButton.Text = 'Disengage';

            % Create STARTButton
            app.STARTButton = uibutton(app.UIFigure, 'push');
            app.STARTButton.ButtonPushedFcn = createCallbackFcn(app, @STARTButtonPushed, true);
            app.STARTButton.BackgroundColor = [0 1 0];
            app.STARTButton.Position = [253 22 82 62];
            app.STARTButton.Text = 'START';

            % Create ModeSwitchLabel
            app.ModeSwitchLabel = uilabel(app.UIFigure);
            app.ModeSwitchLabel.HorizontalAlignment = 'center';
            app.ModeSwitchLabel.Position = [329 237 35 22];
            app.ModeSwitchLabel.Text = 'Mode';

            % Create ModeSwitch
            app.ModeSwitch = uiswitch(app.UIFigure, 'slider');
            app.ModeSwitch.Items = {'Manual', 'Automatic'};
            app.ModeSwitch.Position = [324 207 45 20];
            app.ModeSwitch.Value = 'Manual';

            % Create CartesianControlLabel
            app.CartesianControlLabel = uilabel(app.UIFigure);
            app.CartesianControlLabel.Position = [295 700 99 21];
            app.CartesianControlLabel.Text = 'Cartesian Control';

            % Create VzButton
            app.VzButton = uibutton(app.UIFigure, 'push');
            app.VzButton.ButtonPushedFcn = createCallbackFcn(app, @VzButtonPushed, true);
            app.VzButton.Position = [230 664 33 27];
            app.VzButton.Text = 'Vz-';

            % Create VzButton_2
            app.VzButton_2 = uibutton(app.UIFigure, 'push');
            app.VzButton_2.ButtonPushedFcn = createCallbackFcn(app, @VzButton_2Pushed, true);
            app.VzButton_2.Position = [289 664 36 27];
            app.VzButton_2.Text = 'Vz+';

            % Create VxButton
            app.VxButton = uibutton(app.UIFigure, 'push');
            app.VxButton.ButtonPushedFcn = createCallbackFcn(app, @VxButtonPushed, true);
            app.VxButton.Position = [259 635 36 27];
            app.VxButton.Text = 'Vx+';

            % Create VyButton
            app.VyButton = uibutton(app.UIFigure, 'push');
            app.VyButton.ButtonPushedFcn = createCallbackFcn(app, @VyButtonPushed, true);
            app.VyButton.Position = [231 607 33 27];
            app.VyButton.Text = 'Vy-';

            % Create VyButton_2
            app.VyButton_2 = uibutton(app.UIFigure, 'push');
            app.VyButton_2.ButtonPushedFcn = createCallbackFcn(app, @VyButton_2Pushed, true);
            app.VyButton_2.Position = [289 607 36 27];
            app.VyButton_2.Text = 'Vy+';

            % Create VxButton_2
            app.VxButton_2 = uibutton(app.UIFigure, 'push');
            app.VxButton_2.ButtonPushedFcn = createCallbackFcn(app, @VxButton_2Pushed, true);
            app.VxButton_2.Position = [261 578 33 27];
            app.VxButton_2.Text = 'Vx-';

            % Create WzButton
            app.WzButton = uibutton(app.UIFigure, 'push');
            app.WzButton.ButtonPushedFcn = createCallbackFcn(app, @WzButtonPushed, true);
            app.WzButton.Position = [376 664 36 27];
            app.WzButton.Text = 'Wz-';

            % Create WzButton_2
            app.WzButton_2 = uibutton(app.UIFigure, 'push');
            app.WzButton_2.ButtonPushedFcn = createCallbackFcn(app, @WzButton_2Pushed, true);
            app.WzButton_2.Position = [435 664 39 27];
            app.WzButton_2.Text = 'Wz+';

            % Create WxButton
            app.WxButton = uibutton(app.UIFigure, 'push');
            app.WxButton.ButtonPushedFcn = createCallbackFcn(app, @WxButtonPushed, true);
            app.WxButton.Position = [405 638 39 27];
            app.WxButton.Text = 'Wx+';

            % Create WyButton
            app.WyButton = uibutton(app.UIFigure, 'push');
            app.WyButton.ButtonPushedFcn = createCallbackFcn(app, @WyButtonPushed, true);
            app.WyButton.Position = [376 611 36 27];
            app.WyButton.Text = 'Wy-';

            % Create WyButton_2
            app.WyButton_2 = uibutton(app.UIFigure, 'push');
            app.WyButton_2.ButtonPushedFcn = createCallbackFcn(app, @WyButton_2Pushed, true);
            app.WyButton_2.Position = [435 610 39 27];
            app.WyButton_2.Text = 'Wy+';

            % Create WxButton_2
            app.WxButton_2 = uibutton(app.UIFigure, 'push');
            app.WxButton_2.ButtonPushedFcn = createCallbackFcn(app, @WxButton_2Pushed, true);
            app.WxButton_2.Position = [406 581 36 27];
            app.WxButton_2.Text = 'Wx-';

            % Create VxEditFieldLabel
            app.VxEditFieldLabel = uilabel(app.UIFigure);
            app.VxEditFieldLabel.HorizontalAlignment = 'right';
            app.VxEditFieldLabel.Position = [273 534 25 22];
            app.VxEditFieldLabel.Text = 'Vx';

            % Create VxEditField
            app.VxEditField = uieditfield(app.UIFigure, 'numeric');
            app.VxEditField.Position = [304 533 37 25];

            % Create VyEditFieldLabel
            app.VyEditFieldLabel = uilabel(app.UIFigure);
            app.VyEditFieldLabel.HorizontalAlignment = 'right';
            app.VyEditFieldLabel.Position = [273 501 25 22];
            app.VyEditFieldLabel.Text = 'Vy';

            % Create VyEditField
            app.VyEditField = uieditfield(app.UIFigure, 'numeric');
            app.VyEditField.Position = [304 500 37 25];

            % Create VzEditFieldLabel
            app.VzEditFieldLabel = uilabel(app.UIFigure);
            app.VzEditFieldLabel.HorizontalAlignment = 'right';
            app.VzEditFieldLabel.Position = [273 468 25 22];
            app.VzEditFieldLabel.Text = 'Vz';

            % Create VzEditField
            app.VzEditField = uieditfield(app.UIFigure, 'numeric');
            app.VzEditField.Position = [304 467 37 25];

            % Create VxEditField_2
            app.VxEditField_2 = uieditfield(app.UIFigure, 'numeric');
            app.VxEditField_2.Position = [376 531 37 25];

            % Create VyEditField_2
            app.VyEditField_2 = uieditfield(app.UIFigure, 'numeric');
            app.VyEditField_2.Position = [376 500 37 25];

            % Create VzEditField_2
            app.VzEditField_2 = uieditfield(app.UIFigure, 'numeric');
            app.VzEditField_2.Position = [376 468 37 25];

            % Create VxEditFieldLabel_2
            app.VxEditFieldLabel_2 = uilabel(app.UIFigure);
            app.VxEditFieldLabel_2.HorizontalAlignment = 'right';
            app.VxEditFieldLabel_2.Position = [340 532 25 22];
            app.VxEditFieldLabel_2.Text = 'Wx';

            % Create VyEditFieldLabel_2
            app.VyEditFieldLabel_2 = uilabel(app.UIFigure);
            app.VyEditFieldLabel_2.HorizontalAlignment = 'right';
            app.VyEditFieldLabel_2.Position = [340 501 25 22];
            app.VyEditFieldLabel_2.Text = 'Wy';

            % Create VzEditFieldLabel_2
            app.VzEditFieldLabel_2 = uilabel(app.UIFigure);
            app.VzEditFieldLabel_2.HorizontalAlignment = 'right';
            app.VzEditFieldLabel_2.Position = [340 468 25 22];
            app.VzEditFieldLabel_2.Text = 'Wz';

            % Create RobotToControlSwitchLabel
            app.RobotToControlSwitchLabel = uilabel(app.UIFigure);
            app.RobotToControlSwitchLabel.HorizontalAlignment = 'center';
            app.RobotToControlSwitchLabel.Position = [299 302 95 22];
            app.RobotToControlSwitchLabel.Text = 'Robot To Control';

            % Create RobotToControlSwitch
            app.RobotToControlSwitch = uiswitch(app.UIFigure, 'slider');
            app.RobotToControlSwitch.Items = {'UR3e', 'TM5'};
            app.RobotToControlSwitch.Position = [324 268 45 20];
            app.RobotToControlSwitch.Value = 'UR3e';

            % Create RailLabel
            app.RailLabel = uilabel(app.UIFigure);
            app.RailLabel.HorizontalAlignment = 'right';
            app.RailLabel.Position = [12 675 26 22];
            app.RailLabel.Text = 'Rail';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = projectGUI_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end