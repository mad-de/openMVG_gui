#include <QtWidgets>
#include <QPushButton>
#include <QTextEdit>

#include "software/SfM/omvg_gui.h"

// Initialize paths and filters for later use
QString selfilter_images = "JPEG (*.jpg *.jpeg);;TIFF (*.tif)";

QString parent_path_cut = QDir::currentPath().mid(0,  QDir::currentPath().length()-1);
QString work_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/";

QString initialcommandline_matching = "python workflow.py step=\"matching\" inputpath=\"" + work_dir + "\" camera_model=3";

QString initialcommandline_sfm_solver = "python workflow.py step=\"sfm_solver\" inputpath=\"" + work_dir + "\" imagespath=\"" + work_dir + "\" image1=\"\" image2=\"\" solver=\"1\" ratio=\"0.8\" matrix_filter=\"e\" camera_model=3";

// Initialize stylesheet fix for diasppearing terminal-scrollbar
QString TerminalLikeScrollbar = "QScrollBar:vertical {border: 0px solid black; background-color: #f07b4c; margin: 0px 0px 0px 0px; max-width: 5px;} QScrollBar::handle:vertical {min-height: 0px; background-color: #f07b4c; border: 0px solid black;} QScrollBar::add-line:vertical {border: 0px solid black; height: 0px; subcontrol-position: bottom; subcontrol-origin: margin; background-color: #ffffff;} QScrollBar::sub-line:vertical {border: 0px solid black; height: 0px; subcontrol-position: top; subcontrol-origin: margin; background-color: #ffffff;} QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {border: 0px solid black; width: 0px; height: 0px;} QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {border: 0px solid black;background-color: #300a24;}";

OMVGguiWizard::OMVGguiWizard(QWidget *parent)
    : QWizard(parent)
{
    setPage(Page_Matching, new MatchingPage);
    setPage(Page_Pipeline, new PipelinePage);
    setPage(Page_MVSSelector, new MVSSelectorPage);

    setStartId(Page_Matching);

// !!TODO! IF KANN RAUS? DELETE NEXT THREE LINES
#ifndef Q_OS_MAC
    setWizardStyle(ModernStyle);
#endif
    setOption(HaveHelpButton, true);
    // Generate Preview Button
    setButtonText(QWizard::CustomButton1, tr("&Preview"));
    setOption(QWizard::HaveCustomButton1, true);
    connect(this, SIGNAL(customButtonClicked(int)), this, SLOT(showPreview()));
    setPixmap(QWizard::LogoPixmap, QPixmap(":/images/logo.png"));
    resize(QDesktopWidget().availableGeometry(this).size() * 0.7);

    connect(this, SIGNAL(helpRequested()), this, SLOT(showHelp()));

    setWindowTitle(tr("Open MVG gui for SfM workflow"));
}

void OMVGguiWizard::showHelp()
{
    static QString lastHelpMessage;

    QString message;

    switch (currentId()) {
    case Page_Matching:
        message = tr("This step will process all Images in the selected folder to prepare them for the next steps. In the bottom part of the window you will see the commandline with the command that will be processed. Feel free to change the parameters transmitted to the terminal at a later stage. To start the process Select the folder containing ALL images and press run. Wait until process is finished. If you have already processed the files you want to work on, you can skip this step by clicking Next Button and resume with another step.");
        break;
    case Page_MVSSelector:
        message = tr("phew...");
        break;
    default:
        message = tr("This help is likely not to be of any help.");
    }

    if (lastHelpMessage == message)
        message = tr("Sorry, I already gave what help I could. "
                     "Maybe you should try asking a human?");

    QMessageBox::information(this, tr("Open MVG GUI Help"), message);

    lastHelpMessage = message;
}

void OMVGguiWizard::showPreview()
{
    QString preview_file;

    switch (currentId()) {
        break;
    case Page_Pipeline:
        preview_file = field("Preview_Pipeline").toString();
        break;
    default:
        preview_file = field("Preview_Pipeline").toString();
    }
    QString preview_commandline = "./ply_preview " + preview_file + preview_file;

    process_preview = new QProcess();
    process_preview->start("/bin/bash", QStringList() << "-c" << QString(preview_commandline));
}

// PAGE
// MATCHING
// PAGE

MatchingPage::MatchingPage(QWidget *parent)
    : QWizardPage(parent)
{
    // Set page title and content
    setTitle(tr("Step 1: Image matching & feature computing"));
    setSubTitle(tr("Please select the folder containing your images and press \"Run\""));
   
    // Initialize Widgets

    // Set specific widgets
    InputPath = new QLineEdit(work_dir);
    btnInputPath = new QPushButton("Select");
    InputLabel = new QLabel(tr("Image folder:"));
    // Set general widgets
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox("Advanced Options");
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_matching);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    btnProcess = new QPushButton("Run");
    TerminalMode = new QCheckBox("Terminal Mode (Expert Users)");
    TerminalMode->QWidget::hide();
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    CommandLabel = new QLabel(tr("Output:"));
    input_fields = new QGridLayout;
    CameraSelLabel = new QLabel("[SfMInit_ImageListing] Camera Model:");
    CameraSel = new QComboBox;
    CameraSel->addItem("Pinhole", QVariant(1));
    CameraSel->addItem("Pinhole radial 1", QVariant(2));
    CameraSel->addItem("Pinhole radial 3 (default)", QVariant(3));
    CameraSel->QWidget::hide();
    CameraSelLabel->QWidget::hide();

    // Register fields of vars to use elsewhere.. (don't use an asterisk to not make it mandatory)
    registerField("Matching_InputPath", InputPath);

    // Step specific layout
    input_fields->addWidget(InputLabel, 0, 0);
    input_fields->addWidget(InputPath, 0, 1);
    input_fields->addWidget(btnInputPath, 0, 2);
    // General layout
    AdvancedOptions->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    input_fields->addWidget(AdvancedOptions, 1, 0, 1, 2);
    input_fields->addWidget(CameraSelLabel, 2, 0);
    input_fields->addWidget(CameraSel, 2, 1);
    CameraSel->setCurrentIndex(2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    input_fields->addWidget(TerminalMode, 3, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    input_fields->addWidget(command, 4, 0, 1, 3);
    input_fields->addWidget(CommandLabel, 5, 0);    
    input_fields->addWidget(btnProcess, 5, 2);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    input_fields->addWidget(txtReport, 6, 0, 1, 3);

    // Finalize
    setLayout(input_fields);
    input_fields->setAlignment(Qt::AlignBottom);

    // Connect buttons with processes
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(btnInputPath,SIGNAL(clicked()),this,SLOT(btnInputPathClicked()));
    connect(AdvancedOptions,SIGNAL(clicked()),this,SLOT(btnAdvancedOptionsClicked()));
    connect(TerminalMode,SIGNAL(clicked()),this,SLOT(btnTerminalModeClicked()));
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    connect(CameraSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&MatchingPage::on_CameraSel_changed);
}

int MatchingPage::nextId() const
{
  return OMVGguiWizard::Page_Pipeline;
}

// When Initializing page (use Show event instead of initialize page) set Text to "Skip" or "Next"
void MatchingPage::showEvent(QShowEvent*)
{
    // Have we been here before? Set the buttons accordingly
    if(field("Matching_finished").toString() == "false") {
	wizard()->button(QWizard::NextButton)->setText("Next >");
    }
    else {
    wizard()->button(QWizard::NextButton)->setText("Skip >");
    }	

    // Do we need the cancel button here? I don't think so...
    wizard()->button(QWizard::CancelButton)->QWidget::hide();
    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
}

// Event: Select Input path
void MatchingPage::btnInputPathClicked()
{
    QString str_get_commando;

    str_get_commando = InputPath->text();

	QString InputFolder;
	
	InputFolder = QFileDialog::getExistingDirectory(
    this, 
    tr("Choose folder containing your input images"),
    str_get_commando,
    QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog) + "/";
    // change Folder according to selection in input field
    InputPath->setText(InputFolder);
    // changes inputpath="..." to new inputpath in exec field
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + InputFolder + "\"");
    command->setText(str_commando);

    // Have we been here before? Enable re-running
    if(field("Matching_finished").toString() == "false") {
	btnProcess->setText("Run");
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
}

// Event: Click Run
void MatchingPage::btnProcessClicked()
{
    // Disable Skip button + Run Button
    wizard()->button(QWizard::NextButton)->setEnabled(false);
    btnProcess->setText("working");
    btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(false);

    QString str_command;
    txtReport->clear();
    str_command = command->text();
    process_command = new QProcess();
    process_command->start("/bin/bash", QStringList() << "-c" << QString(str_command));
  
    connect(process_command, SIGNAL(readyReadStandardOutput()),this, SLOT(rightMessage()) );
    connect(process_command, SIGNAL(readyReadStandardError()), this, SLOT(wrongMessage()) );
}

// Handle regular output
void MatchingPage::rightMessage()
{
    QByteArray strdata = process_command->readAllStandardOutput();
    txtReport->moveCursor (QTextCursor::End);
    txtReport->insertPlainText (strdata);
    txtReport->moveCursor (QTextCursor::End);
    if (strdata.contains("Press Next to continue")) {
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Next >");
    btnProcess->setText("Finished");
    registerField("Matching_finished", btnProcess);
    }
}

// Handle error messages
void MatchingPage::wrongMessage()
{
    QByteArray strdata = process_command->readAllStandardError();
    txtReport->setTextColor(Qt::red);
    txtReport->append(strdata);
    // Re-enable Skip + Run Button Button
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Skip >");
    btnProcess->setText("Run");
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

// Event: Advanced Options clicked
void MatchingPage::btnAdvancedOptionsClicked()
{
    if (AdvancedOptions->checkState() == Qt::Checked) {
	//Show all the Advanced Options
	command->setEnabled(true);
	TerminalMode->QWidget::show();
	CameraSel->QWidget::show();
	CameraSelLabel->QWidget::show();
    }
    else {
	// Hide all the Advanced options + command
	TerminalMode->QWidget::hide();
	TerminalMode->QCheckBox::setChecked(false);
	command->setEnabled(false);
	command->QWidget::hide();
	CameraSel->QWidget::hide();
	CameraSelLabel->QWidget::hide();
    }
}

// Event: Terminal Mode clicked
void MatchingPage::btnTerminalModeClicked()
{
    if (TerminalMode->checkState() == Qt::Checked) {
	command->setEnabled(true);
	command->QWidget::show();
    }
    else {
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: Field command changed --> Enable run (Assume, whoever clicks that knows what he's doing)
void MatchingPage::fldcommandClicked()
{
    btnProcess->setText("Run");
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}


// Event: Camera type changed
void MatchingPage::on_CameraSel_changed()
{
    QString get_SelItem = CameraSel->itemData(CameraSel->currentIndex()).toString();

    QString str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp ("camera_model=([^\"]*)"), "camera_model=" + get_SelItem);
    command->setText(str_commando);
}


// PAGE
// Pipelines
// PAGE

PipelinePage::PipelinePage(QWidget *parent)
    : QWizardPage(parent)
{
    // Set page title and content
    setTitle(tr("Step 2: SfM Solver"));
    setSubTitle(tr("Please select the solver you want to use and press \"Run\""));
   
    // Initialize Widgets
    // Set specific widgets
    PipelineSelLabel = new QLabel(tr("Select SfM solver:"));
    PipelineSel = new QComboBox;
    PipelineSel->addItem("Incremental (Standard) - Uses two pictures with best matches)", QVariant(1));
    PipelineSel->addItem("Global", QVariant(2));
    InputPath = new QLineEdit("");
    InputPath->QWidget::hide();
    OutputPath = new QLineEdit("");
    OutputPath->QWidget::hide();
    btnInputPath = new QPushButton("Select");
    btnInputPath->QWidget::hide();
    InputLabel = new QLabel(tr("Matches Path:"));
    InputLabel->QWidget::hide();
    ImagesFolderPath = new QLineEdit("");
    ImagesFolderPath->QWidget::hide();
    btnImagesFolderPath = new QPushButton("Select");
    btnImagesFolderPath->QWidget::hide();
    ImagesFolderLabel = new QLabel(tr("Image Folder:"));
    ImagesFolderLabel->QWidget::hide();
    image_selector_grid_descr = new QLabel("[IncrementalSfM] Select Images with best matches to begin matching:");
    ratioLabel = new QLabel("Ratio:");
    ratioLabel->QWidget::hide();
    sliderRatio = new QSlider(Qt::Horizontal);
    sliderRatio->setFocusPolicy(Qt::StrongFocus);
    sliderRatio->setTickPosition(QSlider::TicksBothSides);
    sliderRatio->setTickInterval(1);
    sliderRatio->setSingleStep(1);
    sliderRatio->setMinimum(0);
    sliderRatio->setMaximum(10);
    sliderRatio->setValue(8);
    sliderRatio->QWidget::hide();
    ratioValue = new QLineEdit("0.8");
    ratioValue->setEnabled(false);
    ratioValue->QWidget::setFixedWidth(100);
    ratioValue->setAlignment(Qt::AlignHCenter);	
    ratioValue->QWidget::hide();
    CameraSelLabel = new QLabel("[IncrementalSfM] Camera Model:");
    CameraSel = new QComboBox;
    CameraSel->addItem("Pinhole", QVariant(1));
    CameraSel->addItem("Pinhole radial 1", QVariant(2));
    CameraSel->addItem("Pinhole radial 3 (default)", QVariant(3));
    CameraSel->QWidget::hide();
    CameraSelLabel->QWidget::hide();
    // Incremental-specific
    solverImage1 = new QLineEdit("");
    solverImage1Label = new QLabel("Image 1:");
    solverImage1Button = new QPushButton("Select");
    solverImage2 = new QLineEdit("");
    solverImage2Label = new QLabel("Image 2:");
    solverImage2Button = new QPushButton("Select");
    // Global-specific
    MatrixSelLabel = new QLabel("[GLOBAL] Matrix Filtering:");
    MatrixSel = new QComboBox;
    MatrixSel->addItem("Essential matrix filtering (same focal length)", QVariant(1));
    MatrixSel->addItem("Fundamental matrix filtering", QVariant(2));
    MatrixSel->addItem("Homography matrix filtering", QVariant(3));
    MatrixSel->QWidget::hide();
    MatrixSelLabel->QWidget::hide();

    // Set general widgets
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox("Advanced Options");
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_sfm_solver);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    btnProcess = new QPushButton("Run");
    TerminalMode = new QCheckBox("Terminal Mode (Advanced Users)");
    TerminalMode->QWidget::hide();
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    CommandLabel = new QLabel(tr("Output:"));

    // Set up main Layout
    main_grid = new QGridLayout;  
    input_fields = new QGridLayout;  
    image_selector_grid = new QGridLayout;  
    terminal_fields = new QGridLayout;     
    advanced_options = new QGridLayout;   

    // Register fields of vars to use elsewhere.. (don't use an asterisk to not make it mandatory)
    registerField("Pipeline_OutputPath", OutputPath);
    StatusPipelinePage = new QLineEdit("init");
    registerField("PipelinePage_status", StatusPipelinePage);
    preview_pipeline = new QLineEdit;
    registerField("Preview_Pipeline", preview_pipeline);

    // Step specific layout
    input_fields->addWidget(PipelineSelLabel, 0 , 0);
    input_fields->addWidget(PipelineSel, 0 , 1);

    // Image Selector
    image_selector_grid_descr->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    image_selector_grid->addWidget(image_selector_grid_descr, 0 , 0, 1, 6);
    image_selector_grid->addWidget(solverImage1, 1 , 1);
    image_selector_grid->addWidget(solverImage1Label, 1 , 0);
    image_selector_grid->addWidget(solverImage1Button, 1 , 2);
    image_selector_grid->addWidget(solverImage2, 1 , 4);
    image_selector_grid->addWidget(solverImage2Label, 1 , 3);
    image_selector_grid->addWidget(solverImage2Button, 1 , 5);

    // Advanced layout
    AdvancedOptions->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(AdvancedOptions, 0, 0, 1, 2);
    advanced_options->addWidget(CameraSelLabel, 2, 0);
    advanced_options->addWidget(CameraSel, 2, 1);
    CameraSel->setCurrentIndex(2);
    advanced_options->addWidget(sliderRatio, 3, 1);
    advanced_options->addWidget(ratioLabel, 3, 0);
    advanced_options->addWidget(ratioValue, 3, 2);
    advanced_options->addWidget(InputLabel, 4, 0);
    advanced_options->addWidget(InputPath, 4, 1);
    advanced_options->addWidget(btnInputPath, 4, 2);
    advanced_options->addWidget(ImagesFolderLabel, 5, 0);
    advanced_options->addWidget(ImagesFolderPath, 5, 1);
    advanced_options->addWidget(btnImagesFolderPath, 5, 2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(TerminalMode, 6, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    advanced_options->addWidget(command, 7, 0, 1, 3);
    // Global specific
    advanced_options->addWidget(MatrixSelLabel, 1, 0);
    advanced_options->addWidget(MatrixSel, 1, 1);

    terminal_fields->addWidget(CommandLabel, 0, 0);  
    terminal_fields->addWidget(btnProcess, 0, 6);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    terminal_fields->addWidget(txtReport, 1, 0, 1, 7);

    // Insert into main Layout
    main_grid->addLayout(input_fields, 0, 0);
    main_grid->addLayout(image_selector_grid, 1, 0);
    main_grid->addLayout(advanced_options, 2, 0);
    main_grid->addLayout(terminal_fields, 3, 0);

    // Finalize
    setLayout(main_grid);

    // Connect buttons with processes
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(btnInputPath,SIGNAL(clicked()),this,SLOT(btnInputPathClicked()));
    connect(btnImagesFolderPath,SIGNAL(clicked()),this,SLOT(btnImagesFolderPathClicked()));
    connect(AdvancedOptions,SIGNAL(clicked()),this,SLOT(btnAdvancedOptionsClicked()));
    connect(TerminalMode,SIGNAL(clicked()),this,SLOT(btnTerminalModeClicked()));
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    connect(solverImage1Button,SIGNAL(clicked()),this,SLOT(on_solverImage1Button_clicked()));
    connect(solverImage2Button,SIGNAL(clicked()),this,SLOT(on_solverImage2Button_clicked()));
    connect(PipelineSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&PipelinePage::on_PipelineSel_changed);
    connect(sliderRatio, SIGNAL(valueChanged(int)),this, SLOT(setRatio(int)));
    connect(MatrixSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&PipelinePage::on_MatrixFilter_changed);
    connect(CameraSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&PipelinePage::on_CameraSel_changed);
}

int PipelinePage::nextId() const
{
  return OMVGguiWizard::Page_MVSSelector;
}

// What happens when we press back? Nuthin...
void PipelinePage::cleanupPage()
{
}
// When Initializing page II
void PipelinePage::showEvent(QShowEvent*)
{
    // Insert field values
    QString input_dir_path = field("Matching_InputPath").toString();
    QString output_dir = input_dir_path.mid(0, input_dir_path.length()-1) + "_out/";
    QString json_dir = output_dir + "matches/";
    QString mvs_dir = output_dir + "reconstruction_sequential/";

    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + json_dir + "\"");
    qDebug() << str_commando.replace(QRegExp ("imagespath=\"([^\"]*)\""), "imagespath=\"" + output_dir + "\"");
    command->setText(str_commando);

    // Do we need the cancel button here? I don't think so...
    wizard()->button(QWizard::CancelButton)->QWidget::hide();

    // Have we been here before?
    if (field("PipelinePage_status").toString() == "init") {
    InputPath->setText(json_dir);
    OutputPath->setText(mvs_dir);
    ImagesFolderPath->setText(input_dir_path);
    StatusPipelinePage->setText("visited");
    registerField("PipelinePage_status", StatusPipelinePage);
    }
}

// Event: Select Input path
void PipelinePage::btnInputPathClicked()
{
    QString str_get_commando;

    str_get_commando = InputPath->text();

    QString InputFolder;
	
    InputFolder = QFileDialog::getExistingDirectory(
    this,
    tr("Choose matching folder (contains the JSON file)"),
    str_get_commando,
    QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog) + "/";

    // change Folder according to selection in input field
    InputPath->setText(InputFolder);
    // changes inputpath="..." to new inputpath in exec field
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + InputFolder + "\"");
    command->setText(str_commando);

    enable_run_again();
}

// Event: Select Images Folder path
void PipelinePage::btnImagesFolderPathClicked()
{
    QString str_get_commando;

    str_get_commando = InputPath->text();

	QString InputFolder;
	
	InputFolder = QFileDialog::getExistingDirectory(
    this, 
    tr("Choose folder containing your input images"),
    str_get_commando,
    QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog) + "/";
    // change Folder according to selection in input field
    ImagesFolderPath->setText(InputFolder);
    // changes inputpath="..." to new inputpath in exec field
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("imagespath=\"([^\"]*)\""), "imagespath=\"" + InputFolder + "\"");
    command->setText(str_commando);

    enable_run_again();
}

// Event: Click Run
void PipelinePage::btnProcessClicked()
{
    QString get_SelItem = PipelineSel->itemData(PipelineSel->currentIndex()).toString();
    // Display warnign if there is a field missing in first pipeline
    if((get_SelItem == "1") && ((solverImage1->text() == "") || (solverImage2->text() == "")))  {
	QMessageBox::warning(this, tr("Error"),
                             tr("Please select the two Pictures with the best matches first"));
    }
    else {
	// Disable Skip button + Run Button
	wizard()->button(QWizard::NextButton)->setEnabled(false);
     	StatusPipelinePage->setText("running");
        registerField("PipelinePage_status", StatusPipelinePage);
	btnProcess->setText("working");
	btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(false);

	QString str_command;
	txtReport->clear();
	str_command = command->text();
	process_command = new QProcess();
	process_command->start("/bin/bash", QStringList() << "-c" << QString(str_command));

	connect(process_command, SIGNAL(readyReadStandardOutput()),this, SLOT(rightMessage()) );
	connect(process_command, SIGNAL(readyReadStandardError()), this, SLOT(wrongMessage()) );
    }
}

// Handle regular output
void PipelinePage::rightMessage()
{
    QByteArray strdata = process_command->readAllStandardOutput();
    QString strdata_qstr = strdata;
    QString strdata_qstr_copy = strdata;
    QString strdata_qstr_output = strdata;
    // When triggered, get the preview_path output, enable Preview button, don't show
    if (strdata.contains("preview_path")) {
	qDebug() << strdata_qstr.replace(QRegExp (".*preview_path"), "" );
	qDebug() << strdata_qstr.replace(QRegExp ("end_path.*"), "" );
	preview_pipeline->setText(strdata_qstr);
	registerField("Preview_Pipeline", preview_pipeline);
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);
	qDebug() << strdata_qstr_output.replace(QRegExp ("preview_path.*end_path"), "" );
	// Also get paths for export options
	qDebug() << strdata_qstr_copy.replace(QRegExp (".*mvs_output_path"), "" );
	qDebug() << strdata_qstr_copy.replace(QRegExp ("end_path.*"), "" );
	OutputPath->setText(strdata_qstr_copy);
    	registerField("Pipeline_OutputPath", OutputPath);
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (strdata_qstr_output);
	txtReport->moveCursor (QTextCursor::End);
    }
    else
    {
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (strdata);
	txtReport->moveCursor (QTextCursor::End);
    }
    // Finished? Hooray! Get this party started:
    if (strdata.contains("Press Next to continue")) {
	wizard()->button(QWizard::NextButton)->setEnabled(true);
	wizard()->button(QWizard::NextButton)->setText("Next >");
	btnProcess->setText("Finished");
	StatusPipelinePage->setText("finished");
	registerField("PipelinePage_status", StatusPipelinePage);
    }
}


// Handle error messages
void PipelinePage::wrongMessage()
{
    QByteArray strdata = process_command->readAllStandardError();
    txtReport->setTextColor(Qt::red);
    txtReport->append(strdata);
    StatusPipelinePage->setText("failed");
    registerField("PipelinePage_status", StatusPipelinePage);
    // Re-enable Skip + Run Button Button
    enable_rerunning();
}

// Event: Advanced Options clicked
void PipelinePage::btnAdvancedOptionsClicked()
{
    if (AdvancedOptions->checkState() == Qt::Checked) {
	//Show all the Advanced Options
	command->setEnabled(true);
	TerminalMode->QWidget::show();
	InputLabel->QWidget::show();
	InputPath->QWidget::show();
	btnInputPath->QWidget::show();
        sliderRatio->QWidget::show();
   	ratioValue->QWidget::show();
	ratioLabel->QWidget::show();
	// check if Pipeline = Global - show Matrix selector
	if (PipelineSel->itemData(PipelineSel->currentIndex()).toString() == "2")
	{
	    MatrixSel->QWidget::show();
	    MatrixSelLabel->QWidget::show();
	}
	// If Pipeline = Incremental show image folder selection
	else
	{
	    ImagesFolderLabel->QWidget::show();
	    ImagesFolderPath->QWidget::show();
	    btnImagesFolderPath->QWidget::show();
	    CameraSel->QWidget::show();
   	    CameraSelLabel->QWidget::show();
	}
    }
    else {
	// Hide all the Advanced options + command (everything)
	InputLabel->QWidget::hide();
	InputPath->QWidget::hide();
	btnInputPath->QWidget::hide();
	ImagesFolderLabel->QWidget::hide();
	ImagesFolderPath->QWidget::hide();
	btnImagesFolderPath->QWidget::hide();
        sliderRatio->QWidget::hide();
   	ratioValue->QWidget::hide();
	ratioLabel->QWidget::hide();
	MatrixSel->QWidget::hide();
	MatrixSelLabel->QWidget::hide();
	CameraSel->QWidget::hide();
	CameraSelLabel->QWidget::hide();

	TerminalMode->QWidget::hide();
	TerminalMode->QCheckBox::setChecked(false);
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: Terminal Mode clicked
void PipelinePage::btnTerminalModeClicked()
{
    if (TerminalMode->checkState() == Qt::Checked) {
	command->setEnabled(true);
	command->QWidget::show();
    }
    else {
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: Field command changed --> Enable run (Assume, whoever clicks that knows what he's doing)
void PipelinePage::fldcommandClicked()
{
    btnProcess->setText("Run");
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

// Event: Select Image1
void PipelinePage::on_solverImage1Button_clicked()
{
    QString str_get_basepath;

    str_get_basepath = field("Matching_InputPath").toString();

    QString file = QFileDialog::getOpenFileName(
    this,
    tr("Choose an image to initialize matching"),
    str_get_basepath,
    selfilter_images
    );

    // get filename

    QFileInfo filepath(file);
    QString filename = filepath.fileName();

    solverImage1->setText(filename);
    // changes images
    QString str_commando = command->text();

    qDebug() << str_commando.replace(QRegExp ("image1=\"([^\"]*)\""), "image1=\"" + filename + "\"");

    command->setText(str_commando);

    enable_run_again();
}

// Event: Select Image2
void PipelinePage::on_solverImage2Button_clicked()
{
    QString str_get_basepath;

    str_get_basepath = field("Matching_InputPath").toString();

    QString file = QFileDialog::getOpenFileName(
    this,
    tr("Choose an image to initialize matching"),
    str_get_basepath,
    selfilter_images
    );

    // get filename

    QFileInfo filepath(file);
    QString filename = filepath.fileName();

    solverImage2->setText(filename);
    // changes images
    QString str_commando = command->text();

    qDebug() << str_commando.replace(QRegExp ("image2=\"([^\"]*)\""), "image2=\"" + filename + "\"");

    command->setText(str_commando);

    enable_run_again();
}

// Event: Pipeline Selector changed
void PipelinePage::on_PipelineSel_changed()
{
    QString get_SelItem = PipelineSel->itemData(PipelineSel->currentIndex()).toString();

    QString str_commando = command->text();

    qDebug() << str_commando.replace(QRegExp ("solver=\"([^\"]*)\""), "solver=\"" + get_SelItem + "\"");
    
    // Dirty: hide non-affected options, show affected
    if(get_SelItem == "2")
    {
	// Show Matrix selector, hide Imagesfolder
	if(AdvancedOptions->checkState() == Qt::Checked)
	{
	    MatrixSel->QWidget::show();
	    MatrixSelLabel->QWidget::show();
	    ImagesFolderLabel->QWidget::hide();
	    ImagesFolderPath->QWidget::hide();
	    btnImagesFolderPath->QWidget::hide();
	    CameraSel->QWidget::hide();
   	    CameraSelLabel->QWidget::hide();
	}
	    solverImage1->QWidget::hide();
	    solverImage1Button->QWidget::hide();
	    solverImage1Label->QWidget::hide();
	    solverImage2->QWidget::hide();
	    solverImage2Button->QWidget::hide();
	    solverImage2Label->QWidget::hide();
            image_selector_grid_descr->QWidget::hide();
    }

    // Dirty: show non-affected options
    if(get_SelItem == "1")
    {
	// Hide Matrix selector + show Images
	if(AdvancedOptions->checkState() == Qt::Checked)
	{
	    MatrixSel->QWidget::hide();
	    MatrixSelLabel->QWidget::hide();
	    ImagesFolderLabel->QWidget::show();
	    ImagesFolderPath->QWidget::show();
	    btnImagesFolderPath->QWidget::show();
	    CameraSel->QWidget::show();
   	    CameraSelLabel->QWidget::show();
	}
	solverImage1->QWidget::show();
	solverImage1Button->QWidget::show();
	solverImage1Label->QWidget::show();
	solverImage2->QWidget::show();
	solverImage2Button->QWidget::show();
	solverImage2Label->QWidget::show();
        image_selector_grid_descr->QWidget::show();
    }
    command->setText(str_commando);

    enable_run_again();
}

// Event: Matrix Filter changed
void PipelinePage::on_MatrixFilter_changed()
{
    QString get_SelItem = MatrixSel->itemData(MatrixSel->currentIndex()).toString();

    QString str_commando = command->text();
    
    // Option 1 = e
    if(get_SelItem == "1")
    {
	qDebug() << str_commando.replace(QRegExp ("matrix_filter=\"([^\"]*)\""), "matrix_filter=\"e\"");
    }
    // Option 2 = f
    if(get_SelItem == "2")
    {
	qDebug() << str_commando.replace(QRegExp ("matrix_filter=\"([^\"]*)\""), "matrix_filter=\"f\"");
    }
    // Option 2 = h
    if(get_SelItem == "3")
    {
	qDebug() << str_commando.replace(QRegExp ("matrix_filter=\"([^\"]*)\""), "matrix_filter=\"h\"");
    }
    command->setText(str_commando);

    enable_run_again();
}

// Event: Ratio changed
void PipelinePage::setRatio(int value)
{
    QString str_commando = command->text();
    if(value == 10) { 
	ratioValue->setText("1"); 
       	qDebug() << str_commando.replace(QRegExp ("ratio=\"([^\"]*)\""), "ratio=\"1\"");
    }
    else { 
    	QString ratioText = QString::number(value);	
	ratioValue->setText("0." + ratioText);
       	qDebug() << str_commando.replace(QRegExp ("ratio=\"([^\"]*)\""), "ratio=\"0." + ratioText + "\"");
    }
    command->setText(str_commando);

    enable_run_again();
}
// Event: Camera type changed
void PipelinePage::on_CameraSel_changed()
{
    QString get_SelItem = CameraSel->itemData(CameraSel->currentIndex()).toString();

    QString str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp ("camera_model=([^\"]*)"), "camera_model=" + get_SelItem);
    command->setText(str_commando);

    enable_run_again();
}

// Enable re-running Selector changed
void PipelinePage::enable_rerunning()
{
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Skip >");
    btnProcess->setText("Run");
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}
// Have we been here before? Enable running again
void PipelinePage::enable_run_again()
{
    if(field("PipelinePage_status").toString() == "finished") {
	btnProcess->setText("Run");
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
}

// PAGE
// Multiple View Sterovision
// PAGE

MVSSelectorPage::MVSSelectorPage(QWidget *parent)
    : QWizardPage(parent)
{
    // Set page title and content
    setTitle(tr("Step 3: Select Multi-View Stereovision workflow"));
    setSubTitle(tr("Please select the Stereovision protocol to use press \"Run\""));
   
    // Initialize Widgets

    // Set general widgets
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox("Advanced Options");
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_matching);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    InputPath = new QLineEdit(work_dir);
    InputPath->QWidget::hide();
    btnInputPath = new QPushButton("Select");
    btnInputPath->QWidget::hide();
    InputLabel = new QLabel(tr("sfm_data.json Folder:"));
    InputLabel->QWidget::hide();
    btnProcess = new QPushButton("Run");
    TerminalMode = new QCheckBox("Terminal Mode (Expert Users)");
    TerminalMode->QWidget::hide();
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    CommandLabel = new QLabel(tr("Output:"));
    input_fields = new QGridLayout;

    // General layout
    AdvancedOptions->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    input_fields->addWidget(AdvancedOptions, 1, 0, 1, 2);
    input_fields->addWidget(InputLabel, 2, 0);
    input_fields->addWidget(InputPath, 2, 1);
    input_fields->addWidget(btnInputPath, 2, 2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    input_fields->addWidget(TerminalMode, 3, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    input_fields->addWidget(command, 4, 0, 1, 3);
    input_fields->addWidget(CommandLabel, 5, 0);    
    input_fields->addWidget(btnProcess, 5, 2);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    input_fields->addWidget(txtReport, 6, 0, 1, 3);

    // Finalize
    setLayout(input_fields);
    input_fields->setAlignment(Qt::AlignBottom);

    // Connect buttons with processes
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(btnInputPath,SIGNAL(clicked()),this,SLOT(btnInputPathClicked()));
    connect(AdvancedOptions,SIGNAL(clicked()),this,SLOT(btnAdvancedOptionsClicked()));
    connect(TerminalMode,SIGNAL(clicked()),this,SLOT(btnTerminalModeClicked()));
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
}

int MVSSelectorPage::nextId() const
{
    return -1;
}


// When Initializing page (use Show event instead of initialize page) set Text to "Skip" or "Next"
void MVSSelectorPage::showEvent(QShowEvent*)
{
    InputPath->setText(field("Pipeline_OutputPath").toString());
    // Have we been here before? Set the buttons accordingly
    if(field("MVS_finished").toString() == "false") {
	wizard()->button(QWizard::NextButton)->setText("Next >");
    }
    else {
    wizard()->button(QWizard::NextButton)->setText("Skip >");
    }	

    // Do we need the cancel button here? I don't think so...
    wizard()->button(QWizard::CancelButton)->QWidget::hide();
    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
}

// Event: Select Input path
void MVSSelectorPage::btnInputPathClicked()
{
    QString str_get_commando;

    str_get_commando = InputPath->text();

	QString InputFolder;
	
	InputFolder = QFileDialog::getExistingDirectory(
    this, 
    tr("Choose folder containing your input images"),
    str_get_commando,
    QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog) + "/";
    // change Folder according to selection in input field
    InputPath->setText(InputFolder);
    // changes inputpath="..." to new inputpath in exec field
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + InputFolder + "\"");
    command->setText(str_commando);

    // Have we been here before? Enable re-running
    if(field("MVS_finished").toString() == "false") {
	btnProcess->setText("Run");
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
}

// Event: Click Run
void MVSSelectorPage::btnProcessClicked()
{
    // Disable Skip button + Run Button
    wizard()->button(QWizard::NextButton)->setEnabled(false);
    btnProcess->setText("working");
    btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(false);

    QString str_command;
    txtReport->clear();
    str_command = command->text();
    process_command = new QProcess();
    process_command->start("/bin/bash", QStringList() << "-c" << QString(str_command));
  
    connect(process_command, SIGNAL(readyReadStandardOutput()),this, SLOT(rightMessage()) );
    connect(process_command, SIGNAL(readyReadStandardError()), this, SLOT(wrongMessage()) );
}

// Handle regular output
void MVSSelectorPage::rightMessage()
{
    QByteArray strdata = process_command->readAllStandardOutput();
    txtReport->moveCursor (QTextCursor::End);
    txtReport->insertPlainText (strdata);
    txtReport->moveCursor (QTextCursor::End);
    if (strdata.contains("Press Next to continue")) {
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Next >");
    btnProcess->setText("Finished");
    registerField("MVS_finished", btnProcess);
    }
}

// Handle error messages
void MVSSelectorPage::wrongMessage()
{
    QByteArray strdata = process_command->readAllStandardError();
    txtReport->setTextColor(Qt::red);
    txtReport->append(strdata);
    // Re-enable Skip + Run Button Button
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Skip >");
    btnProcess->setText("Run");
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

// Event: Advanced Options clicked
void MVSSelectorPage::btnAdvancedOptionsClicked()
{
    if (AdvancedOptions->checkState() == Qt::Checked) {
	//Show all the Advanced Options
	command->setEnabled(true);
	TerminalMode->QWidget::show();
    	InputPath->QWidget::show();
    	btnInputPath->QWidget::show();
    	InputLabel->QWidget::show();
    }
    else {
	// Hide all the Advanced options + command
	TerminalMode->QWidget::hide();
	TerminalMode->QCheckBox::setChecked(false);
	command->setEnabled(false);
	command->QWidget::hide();
    	InputPath->QWidget::hide();
    	btnInputPath->QWidget::hide();
    	InputLabel->QWidget::hide();
    }
}

// Event: Terminal Mode clicked
void MVSSelectorPage::btnTerminalModeClicked()
{
    if (TerminalMode->checkState() == Qt::Checked) {
	command->setEnabled(true);
	command->QWidget::show();
    }
    else {
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: Field command changed --> Enable run (Assume, whoever clicks that knows what he's doing)
void MVSSelectorPage::fldcommandClicked()
{
    btnProcess->setText("Run");
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

