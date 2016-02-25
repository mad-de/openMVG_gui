#ifndef SfM_gui_H
#define SfM_gui_H

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
    enum { Page_Comp_Features, Page_Pipeline, Page_MVSSelector};

    OMVGguiWizard(QWidget *parent = 0);

private slots:
    void showHelp();
    void showPreview();
private:
    QProcess *process_preview;
};

// PAGE: Compute Features

class Comp_FeaturesPage : public QWizardPage
{
    Q_OBJECT

public:
    Comp_FeaturesPage(QWidget *parent = 0);

    void showEvent(QShowEvent*) Q_DECL_OVERRIDE;
    int nextId() const Q_DECL_OVERRIDE;

private slots:
    void finished_demo_download();
    void failed_demo_download();
    void check_demo_path();
    void rightMessage();
    void wrongMessage();
    void btnInputPathClicked();
    void btnAdvancedOptionsClicked();
    void btnProcessClicked();
    void btnTerminalModeClicked();
    void fldcommandClicked();
    void on_CameraSel_changed();
    void on_DescrPres_changed();
    void on_DescrMeth_changed();

private:
    QTimer* demo_download_timer;
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QCheckBox *AdvancedOptions;
    QCheckBox *TerminalMode;
    QTextEdit *txtReport;
    QLineEdit *command;
    QPushButton *btnProcess;
    QLabel *InputLabel;
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *main_grid;
    QGridLayout *advanced_options;
    QGridLayout *input_fields;
    QGridLayout *terminal_fields;
    QLabel *CameraSelLabel;
    QComboBox *CameraSel;
    QLabel *DescrPresLabel;
    QComboBox *DescrPres;
    QLabel *DescrMethLabel;
    QComboBox *DescrMeth;
    QProcess *process_preview_init;
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
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QLineEdit *OutputPath;
    QLineEdit *ImagesFolderPath;
    QPushButton *btnImagesFolderPath;
    QLabel *ImagesFolderLabel;
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

    void cleanupPage();
    void showEvent(QShowEvent*) Q_DECL_OVERRIDE;
    int nextId() const Q_DECL_OVERRIDE;

private slots:
    void rightMessage();
    void wrongMessage();
    void btnInputPathClicked();
    void btnOutputPathClicked();
    void btnAdvancedOptionsClicked();
    void btnProcessClicked();
    void btnTerminalModeClicked();
    void fldcommandClicked();
    void on_MVSSel_changed();
    void btnUseDensifyClicked();
    void btnUseRefineClicked();
    void enable_rerunning();

private:
    QLineEdit *StatusMVSSelectorPage;
    QLabel *InputLabel;
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QLabel *OutputLabel;
    QLineEdit *OutputPath;
    QPushButton *btnOutputPath;
    QLabel *MVSSelLabel;
    QComboBox *MVSSel;
    QCheckBox *AdvancedOptions;
    QCheckBox *TerminalMode;
    QTextEdit *txtReport;
    QLineEdit *command;
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *main_grid;
    QGridLayout *advanced_options;
    QGridLayout *input_fields;
    QGridLayout *terminal_fields;
    QPushButton *btnProcess;
    QCheckBox *UseRefine;
    QCheckBox *UseDensify;
    QLineEdit *preview_mvs;
};

#endif
