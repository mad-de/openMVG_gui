#include <QtWidgets>
#include <QPushButton>
#include <QTextEdit>

#include "software/SfM/SfM_gui.h"

// Initialize paths string and filters for later use
QString selfilter_images = "JPEG (*.jpg *.jpeg);;TIFF (*.tif)";

QString parent_path_cut = QDir::currentPath().mid(0,  QDir::currentPath().length()-1);
QString work_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/";

QString initialcommandline_matching = "python ../software/SfM/workflow.py step=\"matching\" inputpath=\"" + work_dir + "\" camera_model=3 descr_pres=\"NORMAL\" descr_meth=\"SIFT\" force=1";

QString initialcommandline_sfm_solver = "python ../software/SfM/workflow.py step=\"sfm_solver\" inputpath=\"" + work_dir + "\" imagespath=\"" + work_dir + "\" image1=\"\" image2=\"\" solver=\"1\" ratio=\"0.8\" matrix_filter=\"e\" camera_model=3 force=1";

QString initialcommandline_mvs_selector = "python ../software/SfM/workflow.py step=\"openMVS\" inputpath=\"" + work_dir + "\" output_dir=\"" + work_dir + "\" use_densify=\"ON\" use_refine=\"ON\"";

// Define some vars for working on the demo files use - images dir for the check
QProcess *procDemoDl = new QProcess();
QDir demo_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/";
QDir imageset_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/";
QFile Ktxt(parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/K.txt");
QFile Readmetxt (parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/Readme.txt");
QString demo_path (parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/");
// Ugly: use int not to call download over and over again
int visited_matching = 0;

// Initialize stylesheet fix for diasppearing terminal-scrollbar
QString TerminalLikeScrollbar = "QScrollBar:vertical {border: 0px solid black; background-color: #f07b4c; margin: 0px 0px 0px 0px; max-width: 5px;} QScrollBar::handle:vertical {min-height: 0px; background-color: #f07b4c; border: 0px solid black;} QScrollBar::add-line:vertical {border: 0px solid black; height: 0px; subcontrol-position: bottom; subcontrol-origin: margin; background-color: #ffffff;} QScrollBar::sub-line:vertical {border: 0px solid black; height: 0px; subcontrol-position: top; subcontrol-origin: margin; background-color: #ffffff;} QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {border: 0px solid black; width: 0px; height: 0px;} QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {border: 0px solid black;background-color: #300a24;}";

OMVGguiWizard::OMVGguiWizard(QWidget *parent)
    : QWizard(parent)
{
    setPage(Page_Matching, new MatchingPage);
    setPage(Page_Pipeline, new PipelinePage);
    setPage(Page_MVSSelector, new MVSSelectorPage);

    setStartId(Page_Matching);

    // Mac stuff (for multi-platform approach)
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
        message = tr("This step will process all Images in the selected folder to prepare them for the next steps.<br>In the bottom part of the window you will see the commandline with the command that will be processed. Feel free to change the parameters transmitted to the terminal at a later stage. To start the process Select the folder containing ALL images and press run. Wait until process is finished.<br> If you have already processed the files you want to work on, you can skip this step by clicking Next Button and resume with another step.");
        break;
    case Page_Pipeline:
        message = tr("<u>This step will let you chose between one SfM solving options:</u><br><br><b>GlobalACSfM</b> is based on the paper \"Global Fusion of Relative Motions for Robust, Accurate and Scalable Structure from Motion.\" published at ICCV 2013<br>Please use: -p HIGH (describer preset: HIGH option on openMVG_main_ComputeFeatures and -r 0.8 (ratio: 0.8) on openMVG_main_ComputeMatches.<br><br>The<b>ACSfM</b> SfM is an evolution of the implementation used for the paper \"Adaptive Structure from Motion with a contrario model estimation\" published at ACCV 2012.<br>The incremental pipeline is a growing reconstruction process. It starts from an initial two-view reconstruction (the seed) that is iteratively extended by adding new views and 3D points, using pose estimation and triangulation. Due to the incremental nature of the process, successive steps of non-linear refinement, like Bundle Adjustment (BA) and Levenberg-Marquardt steps, are performed to minimize the accumulated error (drift).");
        break;
    case Page_MVSSelector:
        message = tr("<b>OpenMVS</b> allows to compute dense points cloud, surface and textured surfaces of OpenMVG scenes. OpenMVS uses OpenMVG scene thanks to a scene importer.<br><br>OpenMVG exports <b>[PMVS]</b> ready to use project (images, projection matrices and pmvs_options.txt files). If CMVS-PMVS is installed, this program will eun them after exporting.<br><br><b>CMVS</b> is aimed at machines with low memory.<br><br><b>MVE</b> can import a converted openMVG SfM scene and use it to create dense depth map and complete dense 3D models.<br><br>OpenMVG exports <b>[CMPMVS]</b> ready to use project (images, projection matrices and ini configuration file).");
        break;
    }
    QMessageBox::information(this, tr("Open MVG SfM GUI Help"), message);

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
    case Page_MVSSelector:
        preview_file = field("Preview_MVS").toString();
        break;
    default:
        preview_file = field("Preview_Pipeline").toString();
    }
    QFile preview_path(preview_file);
    QString preview_title = tr("Previewing file: ") + QFileInfo(preview_path).fileName();

    QProcess *procPreview = new QProcess();
    QStringList arguments;
    arguments << "-i" <<preview_file << "-t" << preview_title;
    QString preview_command = "./openMVG_SfM_gui_ply_preview";
    procPreview->start(preview_command, arguments);
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
    btnInputPath = new QPushButton(tr("Select"));
    InputLabel = new QLabel(tr("Image folder:"));
    // Set general widgets
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox(tr("Advanced Options"));
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_matching);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    btnProcess = new QPushButton(tr("Run"));
    TerminalMode = new QCheckBox(tr("Terminal Mode (Expert Users)"));
    TerminalMode->QWidget::hide();
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    CommandLabel = new QLabel(tr("Output:"));
    input_fields = new QGridLayout;
    CameraSelLabel = new QLabel(tr("[SfMInit_ImageListing] Camera Model:"));
    CameraSel = new QComboBox;
    CameraSel->addItem(tr("Pinhole"), QVariant(1));
    CameraSel->addItem(tr("Pinhole radial 1"), QVariant(2));
    CameraSel->addItem(tr("Pinhole radial 3 (default)"), QVariant(3));
    CameraSel->QWidget::hide();
    CameraSelLabel->QWidget::hide();
    DescrPresLabel = new QLabel(tr("[SfM_ComputeFeatures] Describer Preset:"));
    DescrPres = new QComboBox;
    DescrPres->addItem(tr("NORMAL"), QVariant(1));
    DescrPres->addItem(tr("HIGH"), QVariant(2));
    DescrPres->addItem(tr("ULTRA"), QVariant(3));
    DescrPres->QWidget::hide();
    DescrPresLabel->QWidget::hide();
    DescrMethLabel = new QLabel(tr("[SfM_ComputeFeatures] Describer Method:"));
    DescrMeth = new QComboBox;
    DescrMeth->addItem(tr("SIFT"), QVariant(1));
    DescrMeth->addItem(tr("AKAZE_FLOAT"), QVariant(2));
    DescrMeth->addItem(tr("AKAZE_MLDB"), QVariant(3));
    DescrMeth->QWidget::hide();
    DescrMethLabel->QWidget::hide();

    // Set up main Layout
    main_grid = new QGridLayout;  
    input_fields = new QGridLayout;   
    advanced_options = new QGridLayout;     
    terminal_fields = new QGridLayout;    

    // Register fields of vars to use elsewhere.. (don't use an asterisk to not make it mandatory)
    registerField("Matching_InputPath", InputPath);

    // Step specific layout
    input_fields->addWidget(InputLabel, 0, 0);
    input_fields->addWidget(InputPath, 0, 1);
    input_fields->addWidget(btnInputPath, 0, 2);
    // General layout
    AdvancedOptions->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(AdvancedOptions, 0, 0, 1, 2);
    advanced_options->addWidget(CameraSelLabel, 1, 0);
    advanced_options->addWidget(CameraSel, 1, 1);
    CameraSel->setCurrentIndex(2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(DescrPresLabel, 2, 0);
    advanced_options->addWidget(DescrPres, 2, 1);
    DescrPres->setCurrentIndex(0);
    advanced_options->addWidget(DescrMethLabel, 3, 0);
    advanced_options->addWidget(DescrMeth, 3, 1);
    DescrPres->setCurrentIndex(0);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(TerminalMode, 4, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    advanced_options->addWidget(command, 5, 0, 1, 3);
    terminal_fields->addWidget(CommandLabel, 0, 0);  
    terminal_fields->addWidget(btnProcess, 0, 8);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    terminal_fields->addWidget(txtReport, 1, 0, 1, 9);

    // Insert into main Layout
    main_grid->addLayout(input_fields, 0, 0);
    main_grid->addLayout(advanced_options, 1, 0);
    main_grid->addLayout(terminal_fields, 2, 0);

    // Finalize
    setLayout(main_grid);

    // Connect buttons with processes
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(btnInputPath,SIGNAL(clicked()),this,SLOT(btnInputPathClicked()));
    connect(AdvancedOptions,SIGNAL(clicked()),this,SLOT(btnAdvancedOptionsClicked()));
    connect(TerminalMode,SIGNAL(clicked()),this,SLOT(btnTerminalModeClicked()));
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    connect(CameraSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&MatchingPage::on_CameraSel_changed);
    connect(DescrPres,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&MatchingPage::on_DescrPres_changed);
    connect(DescrMeth,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&MatchingPage::on_DescrMeth_changed);
}

int MatchingPage::nextId() const
{
  return OMVGguiWizard::Page_Pipeline;
}

// When Initializing page (use Show event instead of initialize page) set Text to "Skip" or "Next"
void MatchingPage::showEvent(QShowEvent*)
{
    if ((QDir(demo_dir).exists() == false) and visited_matching == 0)
     {
	wizard()->button(QWizard::NextButton)->setEnabled(false);
	btnProcess->setText(tr("working"));
	btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(false);
	txtReport->moveCursor (QTextCursor::End);
        txtReport->insertPlainText (tr("You seem to be here for the first time.\nI'll just download the demo files for you... Please be patient..."));
        txtReport->moveCursor (QTextCursor::End);
	QString git_demo = "git clone https://github.com/openMVG/ImageDataset_SceauxCastle.git " + demo_path;
	procDemoDl->start(git_demo);
	// Call finished demo when download is complete
        connect(procDemoDl, SIGNAL(finished(int , QProcess::ExitStatus )), this, SLOT(finished_demo_download()));
        // Set a timer to 20 secs and call if download isn't finished by then...
    	demo_download_timer = new QTimer(this);
    	demo_download_timer->setSingleShot(true);
        demo_download_timer->start(20000);
    	connect(demo_download_timer, SIGNAL(timeout()), SLOT(failed_demo_download()));
        visited_matching = 1;
     }
     else
     {
	check_demo_path();
     }
    // Have we been here before? Set the buttons accordingly
    if(field("Matching_finished").toString() == "false")
    {
	wizard()->button(QWizard::NextButton)->setText(tr("Next >"));
    }
    else 
    {
    wizard()->button(QWizard::NextButton)->setText(tr("Skip >"));
    }	

    // Do we need the cancel button here? I don't think so...
    wizard()->button(QWizard::CancelButton)->QWidget::hide();
    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
}

void MatchingPage::finished_demo_download()
{
    if (QDir(demo_dir).exists() == true)
    {
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (tr(" Download complete!\n"));
	txtReport->moveCursor (QTextCursor::End);
        check_demo_path();	
    } 
}
void MatchingPage::failed_demo_download()
{
    if (QDir(demo_dir).exists() == false)
    {
        // If the download failed delete the initialised folder - otherwise git might fail.
	imageset_dir.removeRecursively();
	// Error output
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (tr(" Download failed.\nPlease check your Internet connection. You can manually download the files by running \"git clone https://github.com/openMVG/ImageDataset_SceauxCastle.git\" in your \"openMVG_build/software/SfM\" folder.\nAlthough the demo files are not available, you can still use this program.\n"));
	txtReport->moveCursor (QTextCursor::End);	
    }    
    check_demo_path();	
}
void MatchingPage::check_demo_path()
{
    if((Ktxt.exists()) or (Readmetxt.exists()))
    {
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (tr("I'm just cleaning up the /images folder for you...\n"));
	txtReport->moveCursor (QTextCursor::End);
	Ktxt.remove();
	Readmetxt.remove();
	if (((Ktxt.exists() == false) and (Readmetxt.exists() == false)))
	     {
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (tr("We're good. Have fun playing around."));
	txtReport->moveCursor (QTextCursor::End);
	     }
	else
	{
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (tr("Arrgh! I failed to delete some extra text files in the /images folder. But you can safely ignore these warnings during matching. Have fun playing around."));
	txtReport->moveCursor (QTextCursor::End);
	}
     }
     wizard()->button(QWizard::NextButton)->setEnabled(true);
     btnProcess->setText(tr("Run"));
     btnProcess->setStyleSheet(tr("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;"));
     btnProcess->setEnabled(true);
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
	btnProcess->setText(tr("Run"));
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
}

// Event: Click Run
void MatchingPage::btnProcessClicked()
{
    // Disable Skip button + Run Button
    wizard()->button(QWizard::NextButton)->setEnabled(false);
    btnProcess->setText(tr("working"));
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
    btnProcess->setText(tr("Finished"));
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
    btnProcess->setText(tr("Run"));
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
	DescrPres->QWidget::show();
	DescrPresLabel->QWidget::show();
	DescrMeth->QWidget::show();
	DescrMethLabel->QWidget::show();
    }
    else {
	// Hide all the Advanced options + command
	TerminalMode->QWidget::hide();
	TerminalMode->QCheckBox::setChecked(false);
	command->setEnabled(false);
	command->QWidget::hide();
	CameraSel->QWidget::hide();
	CameraSelLabel->QWidget::hide();
	DescrPres->QWidget::hide();
	DescrPresLabel->QWidget::hide();
	DescrMeth->QWidget::hide();
	DescrMethLabel->QWidget::hide();
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
    btnProcess->setText(tr("Run"));
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

// Event: Describer Preset changed
void MatchingPage::on_DescrPres_changed()
{
    QString get_SelItem = DescrPres->currentText();

    QString str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp ("descr_pres=\"([^\"]*)\""), "descr_pres=\"" + get_SelItem + "\"");
    command->setText(str_commando);
}

// Event: Describer Method changed
void MatchingPage::on_DescrMeth_changed()
{
    QString get_SelItem = DescrMeth->currentText();

    QString str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp ("descr_meth=\"([^\"]*)\""), "descr_meth=\"" + get_SelItem + "\"");
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
    PipelineSel->addItem(tr("Incremental - Starts with two pictures with the best matches)"), QVariant(1));
    PipelineSel->addItem(tr("Global (Standard)"), QVariant(2));
    InputPath = new QLineEdit("");
    InputPath->QWidget::hide();
    OutputPath = new QLineEdit("");
    OutputPath->QWidget::hide();
    btnInputPath = new QPushButton(tr("Select"));
    btnInputPath->QWidget::hide();
    InputLabel = new QLabel(tr("Matches Path:"));
    InputLabel->QWidget::hide();
    ImagesFolderPath = new QLineEdit("");
    ImagesFolderPath->QWidget::hide();
    btnImagesFolderPath = new QPushButton(tr("Select"));
    btnImagesFolderPath->QWidget::hide();
    ImagesFolderLabel = new QLabel(tr("Image Folder:"));
    ImagesFolderLabel->QWidget::hide();
    image_selector_grid_descr = new QLabel(tr("[IncrementalSfM] Select Images with best matches to begin matching:"));
    image_selector_grid_descr->QWidget::hide();
    ratioLabel = new QLabel(tr("Ratio:"));
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
    CameraSelLabel = new QLabel(tr("[IncrementalSfM] Camera Model:"));
    CameraSel = new QComboBox;
    CameraSel->addItem(tr("Pinhole"), QVariant(1));
    CameraSel->addItem(tr("Pinhole radial 1"), QVariant(2));
    CameraSel->addItem(tr("Pinhole radial 3 (default)"), QVariant(3));
    CameraSel->QWidget::hide();
    CameraSelLabel->QWidget::hide();
    // Incremental-specific
    solverImage1 = new QLineEdit("");
    solverImage1->QWidget::hide();
    solverImage1Label = new QLabel(tr("Image 1:"));
    solverImage1Label->QWidget::hide();
    solverImage1Button = new QPushButton(tr("Select"));
    solverImage1Button->QWidget::hide();
    solverImage2 = new QLineEdit("");
    solverImage2->QWidget::hide();
    solverImage2Label = new QLabel(tr("Image 2:"));
    solverImage2Label->QWidget::hide();
    solverImage2Button = new QPushButton(tr("Select"));
    solverImage2Button->QWidget::hide();
    // Global-specific
    MatrixSelLabel = new QLabel(tr("[GLOBAL] Matrix Filtering:"));
    MatrixSel = new QComboBox;
    MatrixSel->addItem(tr("Essential matrix filtering (same focal length)"), QVariant(1));
    MatrixSel->addItem(tr("Fundamental matrix filtering"), QVariant(2));
    MatrixSel->addItem(tr("Homography matrix filtering"), QVariant(3));
    MatrixSel->QWidget::hide();
    MatrixSelLabel->QWidget::hide();

    // Set general widgets
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox(tr("Advanced Options"));
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_sfm_solver);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    btnProcess = new QPushButton(tr("Run"));
    TerminalMode = new QCheckBox(tr("Terminal Mode (Advanced Users)"));
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
    registerField(tr("Pipeline_OutputPath"), OutputPath);
    StatusPipelinePage = new QLineEdit("init");
    registerField(tr("PipelinePage_status"), StatusPipelinePage);
    preview_pipeline = new QLineEdit;
    registerField(tr("Preview_Pipeline"), preview_pipeline);

    // Step specific layout
    input_fields->addWidget(PipelineSelLabel, 0 , 0);
    input_fields->addWidget(PipelineSel, 0 , 1);
    PipelineSel->setCurrentIndex(1);

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
    terminal_fields->addWidget(btnProcess, 0, 8);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    terminal_fields->addWidget(txtReport, 1, 0, 1, 9);

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
    QString mvs_dir = output_dir + "reconstruction_global/";

    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + json_dir + "\"");
    qDebug() << str_commando.replace(QRegExp ("imagespath=\"([^\"]*)\""), "imagespath=\"" + output_dir + "\"");
    command->setText(str_commando);

    // Do we need the cancel button here? I don't think so...
    wizard()->button(QWizard::CancelButton)->QWidget::hide();

    // Have we been here before? Set the paths accordingly
    if (field("PipelinePage_status").toString() == "init") 
    {
	InputPath->setText(json_dir);
	OutputPath->setText(mvs_dir);
	ImagesFolderPath->setText(input_dir_path);
	StatusPipelinePage->setText("visited");
	registerField("PipelinePage_status", StatusPipelinePage);
    }
    // Have we finished here before? Set the buttons accordingly
    if(field("PipelinePage_status").toString() == "finished") 
    {
	wizard()->button(QWizard::NextButton)->setText(tr("Next >"));
    }
    else 
    {
    	wizard()->button(QWizard::NextButton)->setText(tr("Skip >"));
    }	
    // Is there a Preview path already? Enable Preview button
    if (field("Preview_Pipeline").toString() != "")
    {
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);	
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
	// Disable Skip button + Back Button + Run Button
	wizard()->button(QWizard::NextButton)->setEnabled(false);
	wizard()->button(QWizard::BackButton)->setEnabled(false);
     	StatusPipelinePage->setText("running");
        registerField("PipelinePage_status", StatusPipelinePage);
	btnProcess->setText(tr("working"));
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
	qDebug() << strdata_qstr.replace(QRegExp (".*preview_path "), "" );
	qDebug() << strdata_qstr.replace(QRegExp (" end_path.*"), "" );
	preview_pipeline->setText(strdata_qstr);
	registerField("Preview_Pipeline", preview_pipeline);
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);
	qDebug() << strdata_qstr_output.replace(QRegExp ("preview_path.*end_path"), "" );
	// Also get paths for export options
	qDebug() << strdata_qstr_copy.replace(QRegExp (".*mvs_output_path "), "" );
	qDebug() << strdata_qstr_copy.replace(QRegExp (" end_path.*"), "" );
	OutputPath->setText(strdata_qstr_copy);
    	registerField("Pipeline_OutputPath", OutputPath);
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (strdata_qstr_output);
	txtReport->moveCursor (QTextCursor::End);
     	// Run Preview 
    	QFile preview_path(strdata_qstr);
	QString preview_title = tr("Previewing file: ") + QFileInfo(preview_path).fileName();
	QProcess *procPreview_init = new QProcess();
	QStringList arguments;
	arguments << "-i" << strdata_qstr << "-t" << preview_title;
	QString preview_command = "./openMVG_SfM_gui_ply_preview";
	procPreview_init->start(preview_command, arguments);
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
	wizard()->button(QWizard::BackButton)->setEnabled(true);
	btnProcess->setText(tr("Finished"));
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
    btnProcess->setText(tr("Run"));
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
    wizard()->button(QWizard::BackButton)->setEnabled(true);
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}
// Have we been here before? Enable running again
void PipelinePage::enable_run_again()
{
    if(field("PipelinePage_status").toString() == "finished") {
	btnProcess->setText(tr("Run"));
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
    setTitle(tr("Step 3: Select Multi-View Stereovision workflow / Export format"));
    setSubTitle(tr("Please select the options of your choice and press \"Run\""));
   
    // Initialize Widgets

    // Set general widgets
    MVSSelLabel = new QLabel(tr("Select MVS solver / Export format:"));
    MVSSel = new QComboBox;
    MVSSel->addItem(tr("openMVS (Standard)"), QVariant(1));
    MVSSel->addItem(tr("PMVS"), QVariant(2));
    MVSSel->addItem(tr("CMVS"), QVariant(3));
    MVSSel->addItem(tr("CMPMVS (Export only)"), QVariant(4));
    MVSSel->addItem(tr("MVE (Export only)"), QVariant(5));
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox(tr("Advanced Options"));
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_mvs_selector);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    InputPath = new QLineEdit(work_dir);
    InputPath->QWidget::hide();
    btnInputPath = new QPushButton(tr("Select"));
    btnInputPath->QWidget::hide();
    InputLabel = new QLabel(tr("sfm_data.json Folder:"));
    InputLabel->QWidget::hide();
    OutputPath = new QLineEdit(work_dir);
    OutputPath->QWidget::hide();
    btnOutputPath = new QPushButton(tr("Select"));
    btnOutputPath->QWidget::hide();
    OutputLabel = new QLabel(tr("Output Folder:"));
    OutputLabel->QWidget::hide();
    btnProcess = new QPushButton(tr("Run"));
    TerminalMode = new QCheckBox(tr("Terminal Mode (Expert Users)"));
    TerminalMode->QWidget::hide();
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    CommandLabel = new QLabel(tr("Output:"));
    UseDensify = new QCheckBox(tr("[openmVS] Densify Point Cloud"));
    UseDensify->QWidget::hide();
    UseDensify->QAbstractButton::setChecked(true);
    UseRefine = new QCheckBox(tr("[openmVS] Refine Mesh"));
    UseRefine->QWidget::hide();
    UseRefine->QAbstractButton::setChecked(true);
    preview_mvs = new QLineEdit();
    input_fields = new QGridLayout;
    advanced_options = new QGridLayout;
    terminal_fields = new QGridLayout;
    main_grid = new QGridLayout;

    // General layout
    input_fields->addWidget(MVSSelLabel, 0 , 0);
    input_fields->addWidget(MVSSel, 0 , 1);
    AdvancedOptions->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(AdvancedOptions, 1, 0, 1, 2);
    UseDensify->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(UseDensify, 2, 0, 1, 2);
    UseRefine->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(UseRefine, 3, 0, 1, 2);
    advanced_options->addWidget(InputLabel, 4, 0);
    advanced_options->addWidget(InputPath, 4, 1);
    advanced_options->addWidget(btnInputPath, 4, 2);
    advanced_options->addWidget(OutputLabel, 5, 0);
    advanced_options->addWidget(OutputPath, 5, 1);
    advanced_options->addWidget(btnOutputPath, 5, 2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(TerminalMode, 6, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    advanced_options->addWidget(command, 7, 0, 1, 3);
    terminal_fields->addWidget(CommandLabel, 0, 0);  
    terminal_fields->addWidget(btnProcess, 0, 8);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    terminal_fields->addWidget(txtReport, 1, 0, 1, 9);

    // Insert into main Layout
    main_grid->addLayout(input_fields, 0, 0);
    main_grid->addLayout(advanced_options, 2, 0);
    main_grid->addLayout(terminal_fields, 3, 0);

    // Finalize
    setLayout(main_grid);

    // Connect buttons with processes
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(btnInputPath,SIGNAL(clicked()),this,SLOT(btnInputPathClicked()));
    connect(btnOutputPath,SIGNAL(clicked()),this,SLOT(btnOutputPathClicked()));
    connect(AdvancedOptions,SIGNAL(clicked()),this,SLOT(btnAdvancedOptionsClicked()));
    connect(TerminalMode,SIGNAL(clicked()),this,SLOT(btnTerminalModeClicked()));
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    connect(MVSSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged),this,&MVSSelectorPage::on_MVSSel_changed);
    connect(UseDensify,SIGNAL(clicked()),this,SLOT(btnUseDensifyClicked()));
    connect(UseRefine,SIGNAL(clicked()),this,SLOT(btnUseRefineClicked()));
}

int MVSSelectorPage::nextId() const
{
    return -1;
}

// What happens when we press back? Nuthin...
void MVSSelectorPage::cleanupPage()
{
}

// When Initializing page (use Show event instead of initialize page) set Text to "Skip" or "Next"
void MVSSelectorPage::showEvent(QShowEvent*)
{
    // Have we been here before? Set the buttons accordingly
    if(field("MVS_finished").toString() == "Finished") {
    	wizard()->button(QWizard::FinishButton)->setEnabled(true);
    }
    else
    {
    	wizard()->button(QWizard::FinishButton)->setEnabled(false);
    }
    // set inputpath + outputpath accordingly
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + field("Pipeline_OutputPath").toString() + "\"");
    QString input_path_cut = field("Pipeline_OutputPath").toString().mid(0,  field("Pipeline_OutputPath").toString().length()-1);
    QString output_dir = input_path_cut.mid(0, input_path_cut.lastIndexOf("/")) + "/MVS_out/";
    qDebug() << str_commando.replace(QRegExp ("output_dir=\"([^\"]*)\""), "output_dir=\"" + output_dir + "\"");
    command->setText(str_commando);

    InputPath->setText(field("Pipeline_OutputPath").toString());
    OutputPath->setText(output_dir);

    // Do we need the cancel button here? I don't think so...
    wizard()->button(QWizard::CancelButton)->QWidget::hide();
    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
    // Is there a Preview path already? Enable Preview button
    if (field("Preview_MVS").toString() != "")
    {
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);	
    }
}

// Enable re-running
void MVSSelectorPage::enable_rerunning()
{
    if (field("MVS_finished").toString() == "Finished") {
	btnProcess->setText(tr("Run"));
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
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
    enable_rerunning();
}

// Event: SelectOutput path
void MVSSelectorPage::btnOutputPathClicked()
{
    QString str_get_commando;

    str_get_commando = OutputPath->text();

	QString OutputFolder;
	
	OutputFolder = QFileDialog::getExistingDirectory(
    this, 
    tr("Choose output folder"),
    str_get_commando,
    QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog) + "/";
    // change Folder according to selection in output field
    OutputPath->setText(OutputFolder);
    // changes outputpath="..." to new outputpath in exec field
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("output_dir=\"([^\"]*)\""), "output_dir=\"" + OutputFolder + "\"");
    command->setText(str_commando);

    // Enable re_run
    enable_rerunning();
}

// Event: Click Run
void MVSSelectorPage::btnProcessClicked()
{
    // Disable Skip button + Run Button + Preview button
    wizard()->button(QWizard::FinishButton)->setEnabled(false);
    wizard()->button(QWizard::BackButton)->setEnabled(false);
    btnProcess->setText(tr("working"));
    btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(false);
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);

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
    QString strdata_qstr = strdata;
    QString strdata_qstr_output = strdata;
    if (strdata.contains("preview_path")) {
	qDebug() << strdata_qstr.replace(QRegExp (".*preview_path "), "" );
	qDebug() << strdata_qstr.replace(QRegExp (" end_path.*"), "" );
	preview_mvs->setText(strdata_qstr);
	registerField("Preview_MVS", preview_mvs);
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);
	qDebug() << strdata_qstr_output.replace(QRegExp ("preview_path.*end_path"), "" );
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (strdata_qstr_output);
	txtReport->moveCursor (QTextCursor::End);
     	// Run Preview 
    	QFile preview_path(strdata_qstr);
	QString preview_title = tr("Previewing file: ") + QFileInfo(preview_path).fileName();
	QProcess *procPreview_init = new QProcess();
	QStringList arguments;
	arguments << "-i" << strdata_qstr << "-t" << preview_title;
	QString preview_command = "./openMVG_SfM_gui_ply_preview";
	procPreview_init->start(preview_command, arguments);
    }
    else
    {
	txtReport->moveCursor (QTextCursor::End);
    	txtReport->insertPlainText (strdata);
    	txtReport->moveCursor (QTextCursor::End);
    }
    if (strdata.contains("Press Finish to close")) {
    	wizard()->button(QWizard::FinishButton)->setEnabled(true);
	wizard()->button(QWizard::BackButton)->setEnabled(true);
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
    // Re-enable Back + Run Button Button
    wizard()->button(QWizard::BackButton)->setEnabled(true);
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

// Event: Advanced Options clicked
void MVSSelectorPage::btnAdvancedOptionsClicked()
{
    QString get_SelItem = MVSSel->itemData(MVSSel->currentIndex()).toString();

    QString str_commando = command->text();

    qDebug() << str_commando.replace(QRegExp ("mvs=\"([^\"]*)\""), "mvs=\"" + get_SelItem + "\"");

    if (AdvancedOptions->checkState() == Qt::Checked) {
	//Show all the Advanced Options
	command->setEnabled(true);
	TerminalMode->QWidget::show();
    	InputPath->QWidget::show();
    	btnInputPath->QWidget::show();
    	InputLabel->QWidget::show();
    	OutputPath->QWidget::show();
    	btnOutputPath->QWidget::show();
    	OutputLabel->QWidget::show();
	// Show sepcific options
        if(get_SelItem == "2")
        {
   	    UseDensify->QWidget::hide();
   	    UseRefine->QWidget::hide();
        }
	else if (get_SelItem == "1")
        {
   	    UseDensify->QWidget::show();
   	    UseRefine->QWidget::show();
        }
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
    	OutputPath->QWidget::hide();
    	btnOutputPath->QWidget::hide();
    	OutputLabel->QWidget::hide();
   	UseDensify->QWidget::hide();
   	UseRefine->QWidget::hide();
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
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

// Event: Selector changed
void MVSSelectorPage::on_MVSSel_changed()
{
    QString get_SelItem = MVSSel->itemData(MVSSel->currentIndex()).toString();

    QString str_commando = command->text();

    if(get_SelItem == "1")
    {
	// Hide Matrix selector + show Images
	if(AdvancedOptions->checkState() == Qt::Checked)
	{
   	    UseDensify->QWidget::show();
   	    UseRefine->QWidget::show();
	}
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"openMVS\"");
    }
    else if(get_SelItem == "2")
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"pmvs\"");
    }
    else if(get_SelItem == "3")
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"cmvs\"");
    }
    else if(get_SelItem == "4")
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"cmpmvs\"");
    }
    else if(get_SelItem == "5")
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"mve\"");
    }
    enable_rerunning();
    command->setText(str_commando);
}

// Event btnUseDensify Clicked
void MVSSelectorPage::btnUseDensifyClicked()
{
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    if(UseDensify->checkState() == Qt::Checked)
    {
    qDebug() << str_commando.replace(QRegExp ("use_densify=\"([^\"]*)\""), "use_densify=\"ON\"");
    }
    else
    {
    qDebug() << str_commando.replace(QRegExp ("use_densify=\"([^\"]*)\""), "use_densify=\"OFF\"");
    }
    command->setText(str_commando);
}

// Event btnUseRefine Clicked
void MVSSelectorPage::btnUseRefineClicked()
{
    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    if(UseRefine->checkState() == Qt::Checked)
    {
    qDebug() << str_commando.replace(QRegExp ("use_refine=\"([^\"]*)\""), "use_refine=\"ON\"");
    }
    else
    {
    qDebug() << str_commando.replace(QRegExp ("use_refine=\"([^\"]*)\""), "use_refine=\"OFF\"");
    }
    command->setText(str_commando);
}
