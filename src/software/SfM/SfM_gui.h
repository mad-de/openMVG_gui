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
    void cancelProcess();
    void finished_demo_download();
    void failed_demo_download();
    void check_demo_path();
    void rightMessage();
    void wrongMessage();
    void btnAdvancedOptionsClicked(int checkstate);
    void btnTerminalModeClicked(int checkstate);
    void btnProcessClicked();
    void fldcommandClicked();
    void on_selectors_changed(QString selection_string, QString option_decl);
    void enable_run_again();
    void btnPathbuttonsClicked(QString mode);

private:
    QTimer* demo_download_timer;
    QLabel *InputLabel;
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QCheckBox *AdvancedOptions;
    QCheckBox *TerminalMode;
    QLabel *OutputLabel;
    QLineEdit *OutputPath_finished;
    QLineEdit *OutputPath;
    QPushButton *btnOutputPath;
    QTextEdit *txtReport;
    QLineEdit *command;
    QPushButton *btnProcess;
    QPushButton *btnCancel;
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *main_grid;
    QGridLayout *advanced_options;
    QGridLayout *input_fields;
    QGridLayout *terminal_fields;
    QLabel *CameraSelLabel;
    QComboBox *CameraSel;
    QLabel *GroupCameraSelLabel;
    QComboBox *GroupCameraSel;
    QLabel *DescrPresLabel;
    QComboBox *DescrPres;
    QLabel *DescrMethLabel;
    QComboBox *DescrMeth;
    QLabel *UprightLabel;
    QComboBox *Upright;
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
    void cancelProcess();
    void rightMessage();
    void wrongMessage();
    void btnProcessClicked();
    void btnAdvancedOptionsClicked(int checkstate);
    void btnTerminalModeClicked(int checkstate);
    void fldcommandClicked();
    void setRatio(int value);
    void enable_rerunning();
    void enable_run_again();
    void btnPathbuttonsClicked(QString mode);
    void on_selectors_changed(int selection_num, QString option_decl);

private:
    QLineEdit *StatusPipelinePage;
    QLineEdit *InputPath;
    QPushButton *btnInputPath;
    QLineEdit *OutputPath_Pipeline;
    QLabel *OutputLabel;
    QLineEdit *OutputPath;
    QPushButton *btnOutputPath;
    QLineEdit *OptionsPipeline;
    QLineEdit *ImagesFolderPath;
    QPushButton *btnImagesFolderPath;
    QLabel *ImagesFolderLabel;
    QPushButton *btnProcess;
    QPushButton *btnCancel;
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
    Q_OBJECT;

public:
    MVSSelectorPage(QWidget *parent = 0);

    void cleanupPage();
    void showEvent(QShowEvent*) Q_DECL_OVERRIDE;
    int nextId() const Q_DECL_OVERRIDE;

    QLineEdit *command;

private slots:
    void cancelProcess();
    void fldcommandClicked();
    void btnProcessClicked();
    void rightMessage();
    void wrongMessage();
    void enable_rerunning();
    void PMVSoptionsdisplay();
    void PMVSoptionshide();
    void openMVSoptionsdisplay();
    void openMVSoptionshide();
    void get_standard_paths(QString str_commando);
    void btnPathbuttonsClicked(QString mode);
    void btnTerminalModeClicked(int checkstate);
    void btnAdvancedOptionsClicked(int selection_num);
    void on_MVSSel_changed(int selection_num);
    void btnUseopenMVSoptionsClicked(int checkstate, QString option_decl);
    void pmvsOptionsclicked(QString new_content, QString option_decl);
    void btnOptionsbuttonsClicked(QString mode);
    void getDialogResults(QString mode, QString new_content);

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
    QLabel *CommandLabel;
    QProcess *process_command;
    QGridLayout *main_grid;
    QGridLayout *advanced_options;
    QGridLayout *input_fields;
    QGridLayout *terminal_fields;
    QPushButton *btnProcess;
    QPushButton *btnCancel;
    QLabel *DensifyLabel;
    QCheckBox *UseRefine;
    QCheckBox *UseDensify;
    QLabel *ReconstructLabel;
    QLabel *RefineLabel;
    QLabel *TextureLabel;
    QPushButton *DensifyOptionsButton;
    QPushButton *ReconstructOptionsButton;
    QPushButton *RefineOptionsButton;
    QPushButton *TextureOptionsButton;
    QLineEdit *preview_mvs;
    QLineEdit *OptionsMVS;
    QLabel *ImageCountLabel;
    QLineEdit *ImageCount;
    QLabel *numCPULabel;
    QLineEdit *numCPU;
    QLabel *LevelLabel;
    QLineEdit *level;
    QLabel *csizeLabel;
    QLineEdit *csize;
    QLabel *thresholdLabel;
    QLineEdit *threshold;
    QLabel *wsizeLabel;
    QLineEdit *wsize;
    QLabel *minImageLabel;
    QLineEdit *minImage;
};

// openMVS Dialog

class openMVSDialog : public QDialog
{
     Q_OBJECT

public:
     openMVSDialog(QWidget *parent = 0);

signals:
    void resultAvailable(QString new_content, QString mode);

private slots:
    void checkboxClicked(int checkstate, QString mode);
    void valueChanged(QString mode, QString new_content);

private:
     QGridLayout *mainLayout;
     QCheckBox *RT_use_cglowdensity;
     QLabel *RE_scalesLabel;
     QLineEdit *RE_scales;
     QLabel *RE_resolutionlevelLabel;
     QLineEdit *RE_resolutionlevel;
     QLabel *TE_resolutionlevelLabel;
     QLineEdit *TE_resolutionlevel;
};

#endif
