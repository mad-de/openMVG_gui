#ifndef omvg_gui_H
#define omvg_gui_H

#include <QWizard>
#include <QtWidgets>

class QCheckBox;
class QLabel;
class QLineEdit;
class QRadioButton;

class OMVGguiWizard : public QWizard
{
    Q_OBJECT

public:
    enum { Page_Matching, Page_Pipeline, Page_MVSSelector};

    OMVGguiWizard(QWidget *parent = 0);

private slots:
    void showHelp();
    void showPreview();
private:
    QProcess *process_preview;
};

// PAGE: Matching

class MatchingPage : public QWizardPage
{
    Q_OBJECT

public:
    MatchingPage(QWidget *parent = 0);

    void showEvent(QShowEvent*) Q_DECL_OVERRIDE;
    int nextId() const Q_DECL_OVERRIDE;
 /*   void setVisible(bool visible) Q_DECL_OVERRIDE; */

private slots:
    void rightMessage();
    void wrongMessage();
    void btnInputPathClicked();
    void btnAdvancedOptionsClicked();
    void btnProcessClicked();
    void btnTerminalModeClicked();
    void fldcommandClicked();
    void on_CameraSel_changed();

private:
    QLabel *bottomLabel; // !TODO! DELETE?
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QTextEdit *commandline; // !TODO! DOESNT EXIST - DELETE?
    QCheckBox *AdvancedOptions;
    QCheckBox *TerminalMode;
    QTextEdit *txtReport;
    QLineEdit *command;
    QLabel *InputLabel;
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *input_fields;
    QPushButton *btnProcess;
    QLabel *CameraSelLabel;
    QComboBox *CameraSel;
};

// PAGE: Pipeline

class PipelinePage : public QWizardPage
{
    Q_OBJECT

public:
    PipelinePage(QWidget *parent = 0);

    void cleanupPage();
    void showEvent(QShowEvent*) Q_DECL_OVERRIDE;
    int nextId() const Q_DECL_OVERRIDE;

private slots:
    void rightMessage();
    void wrongMessage();
    void btnInputPathClicked();
    void btnImagesFolderPathClicked();
    void btnProcessClicked();
    void btnAdvancedOptionsClicked();
    void btnTerminalModeClicked();
    void fldcommandClicked();
    void on_solverImage1Button_clicked();
    void on_solverImage2Button_clicked();
    void on_PipelineSel_changed();
    void on_MatrixFilter_changed();
    void setRatio(int value);
    void on_CameraSel_changed();
    void enable_rerunning();
    void enable_run_again();

private:
    QLineEdit *StatusPipelinePage;
    QLabel *bottomLabel; // !TODO! DELETE?
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QLineEdit *OutputPath;
    QLineEdit *ImagesFolderPath;
    QPushButton *btnImagesFolderPath;
    QLabel *ImagesFolderLabel;
    QTextEdit *commandline; // !TODO! DOESNT EXIST - DELETE?
    QPushButton *btnProcess;
    QLabel *PipelineSelLabel;
    QComboBox *PipelineSel;
    QCheckBox *AdvancedOptions;
    QCheckBox *TerminalMode;
    QTextEdit *txtReport;
    QLineEdit *command;
    QLabel *InputLabel;
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *main_grid;
    QGridLayout *advanced_options;
    QGridLayout *input_fields;
    QGridLayout *terminal_fields;
    QGridLayout *image_selector_grid;
    QWidget *image_selector_layout; 
    QLabel *image_selector_grid_descr;
    QSlider *sliderRatio;
    QLabel *ratioLabel;
    QLineEdit *ratioValue;
    QLabel *CameraSelLabel;
    QComboBox *CameraSel;
    QLineEdit *solverImage1;
    QLabel *solverImage1Label;
    QPushButton *solverImage1Button;
    QLineEdit *solverImage2;
    QLabel *solverImage2Label;
    QPushButton *solverImage2Button;
    QLabel *MatrixSelLabel;
    QComboBox *MatrixSel;
    QLineEdit *preview_pipeline;


};

// PAGE: MVSSelector

class MVSSelectorPage : public QWizardPage
{
    Q_OBJECT

public:
    MVSSelectorPage(QWidget *parent = 0);

    void showEvent(QShowEvent*) Q_DECL_OVERRIDE;
    int nextId() const Q_DECL_OVERRIDE;
 /*   void setVisible(bool visible) Q_DECL_OVERRIDE; */

private slots:
    void rightMessage();
    void wrongMessage();
    void btnInputPathClicked();
    void btnAdvancedOptionsClicked();
    void btnProcessClicked();
    void btnTerminalModeClicked();
    void fldcommandClicked();

private:
    QLabel *bottomLabel; // !TODO! DELETE?
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QTextEdit *commandline; // !TODO! DOESNT EXIST - DELETE?
    QCheckBox *AdvancedOptions;
    QCheckBox *TerminalMode;
    QTextEdit *txtReport;
    QLineEdit *command;
    QLabel *InputLabel;
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *input_fields;
    QPushButton *btnProcess;
};

#endif
