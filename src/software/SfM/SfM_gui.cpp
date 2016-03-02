#include <QtWidgets>
#include <QPushButton>
#include <QTextEdit>
#include <thread>

#include "software/SfM/SfM_gui.h"

// Initialize paths string and filters for later use
QString selfilter_images = "JPEG (*.jpg *.jpeg);;TIFF (*.tif)";

QString parent_path_cut = QDir::currentPath().mid(0,  QDir::currentPath().length()-1);
QString work_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/";
QString outputpath = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images_out/matches/";

QString initialcommandline_comp_features = "python ../software/SfM/workflow.py step=\"comp_features\" inputpath=\"" + work_dir + "\" outputpath=\"" + outputpath + "\" camera_model=\"3\" descr_pres=\"NORMAL\" descr_meth=\"SIFT\" force=1";

QString initialcommandline_sfm_solver = "python ../software/SfM/workflow.py step=\"sfm_solver\" inputpath=\"" + work_dir + "\" imagespath=\"" + work_dir + "\" matchespath=\"" + work_dir + "\" outputpath=\"" + work_dir + "\"  image1=\"\" image2=\"\" solver=\"1\" ratio=\"0.8\" matrix_filter=\"e\" camera_model=\"3\" force=1";

QString initialcommandline_mvs_openMVS = "python ../software/SfM/workflow.py step=\"openMVS\" inputpath=\"" + work_dir + "\" output_dir=\"" + work_dir + "\" use_densify=\"ON\" use_refine=\"ON\"";

QString initialcommandline_mvs_CMVS = "python ../software/SfM/workflow.py step=\"cmvs\" inputpath=\"" + work_dir + "\" output_dir=\"" + work_dir + "\" max_imagecount=\"100\" cpu=\"6\" level=\"1\" csize=\"2\" threshold=\"0.7\" wsize=\"7\" minImageNum=\"3\"";

QString initialcommandline_mvs_stand = "python ../software/SfM/workflow.py step=\"curr\" inputpath=\"" + work_dir + "\" output_dir=\"" + work_dir + "\"";

// Define some vars for working on the demo files use - images dir for the check
QProcess *procDemoDl = new QProcess();
QDir demo_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/";
QDir imageset_dir = parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/";
QFile Ktxt(parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/K.txt");
QFile Readmetxt (parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/images/Readme.txt");
QString demo_path (parent_path_cut.mid(0, parent_path_cut.lastIndexOf("/")) + "/software/SfM/ImageDataset_SceauxCastle/");
// Ugly: use int not to call download over and over again
int visited_comp_features = 0;

// Initialize stylesheet fix for diasppearing terminal-scrollbar
QString TerminalLikeScrollbar = "QScrollBar:vertical {border: 0px solid black; background-color: #f07b4c; margin: 0px 0px 0px 0px; max-width: 5px;} QScrollBar::handle:vertical {min-height: 0px; background-color: #f07b4c; border: 0px solid black;} QScrollBar::add-line:vertical {border: 0px solid black; height: 0px; subcontrol-position: bottom; subcontrol-origin: margin; background-color: #ffffff;} QScrollBar::sub-line:vertical {border: 0px solid black; height: 0px; subcontrol-position: top; subcontrol-origin: margin; background-color: #ffffff;} QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {border: 0px solid black; width: 0px; height: 0px;} QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {border: 0px solid black;background-color: #300a24;}";

// Global functions
QString get_num_of_CPUs()
{
    // Estimate how many threads we can run
    unsigned int sys_numCPUs = std::thread::hardware_concurrency();
    QString get_numCPUs = QString::number(sys_numCPUs);
    if (get_numCPUs == "0") { get_numCPUs = "6"; }
    return get_numCPUs;
}

void launchPreview(QString preview_file, QString title, QString options)
{
    QFile preview_path(preview_file);
    QString preview_title = "Previewing file: " + QFileInfo(preview_path).fileName() + title;

    QProcess *procPreview = new QProcess();
    QStringList arguments;
    arguments << "-i" <<preview_file << "-t" << preview_title << "-o" << options;
    QString preview_command = "./openMVG_SfM_gui_ply_preview";
    procPreview->start(preview_command, arguments);
}

OMVGguiWizard::OMVGguiWizard(QWidget *parent)
    : QWizard(parent)
{
    setPage(Page_Comp_Features, new Comp_FeaturesPage);
    setPage(Page_Pipeline, new PipelinePage);
    setPage(Page_MVSSelector, new MVSSelectorPage);

    setStartId(Page_Comp_Features);

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
    setOption(QWizard::NoCancelButton, true);
    // set Window title
    setWindowTitle(tr("Open MVG gui for SfM workflow"));
}

void OMVGguiWizard::showHelp()
{
    static QString lastHelpMessage;

    QString message;

    switch (currentId()) {
    case Page_Comp_Features:
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
    QString title;
    QString options;

    switch (currentId()) {
        break;
    case Page_Pipeline:
        preview_file = field("Preview_Pipeline").toString();
	title = " (Step: Pipeline)";
	options = field("Pipeline_Options").toString();
        break;
    case Page_MVSSelector:
        preview_file = field("Preview_MVS").toString();
	title = " (Step: MVS)";
	options = field("MVS_Options").toString();
        break;
    }
    launchPreview(preview_file, title, options);
}

// PAGE
// Compute Features
// PAGE

Comp_FeaturesPage::Comp_FeaturesPage(QWidget *parent)
    : QWizardPage(parent)
{
    // Set page title and content
    setTitle(tr("Step 1: Image listing & feature computing"));
    setSubTitle(tr("Please select the folder containing your images and press \"Run\""));
   
    // Initialize Widgets

    InputPath = new QLineEdit(work_dir);
    btnInputPath = new QPushButton(tr("Select"));
    InputLabel = new QLabel(tr("Image folder:"));
    // Set general widgets
    OutputPath = new QLineEdit(outputpath);
    btnOutputPath = new QPushButton(tr("Select"));
    OutputLabel = new QLabel(tr("Output folder:"));
    OutputPath->QWidget::hide();
    btnOutputPath->QWidget::hide();
    OutputLabel->QWidget::hide();
    txtReport = new QTextEdit("");
    txtReport->verticalScrollBar()->setStyleSheet(TerminalLikeScrollbar);     // when setting background-color, qt somehow looses all stylesheet info about vertical scrollbar. set it new.
    txtReport->setStyleSheet("background-color: #300a24; border: 0px solid black; color: #ffffff; font: 10pt Monospace;");
    AdvancedOptions = new QCheckBox(tr("Advanced Options"));
    command = new QLineEdit("init");
    command = new QLineEdit(initialcommandline_comp_features);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    btnProcess = new QPushButton(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnCancel = new QPushButton(tr("Cancel"));
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(false);
    TerminalMode = new QCheckBox(tr("Terminal Mode (Expert Users)"));
    TerminalMode->QWidget::hide();
    CommandLabel = new QLabel(tr("Output:"));
    input_fields = new QGridLayout;
    CameraSelLabel = new QLabel(tr("[SfMInit_ImageListing] Camera Model:"));
    CameraSel = new QComboBox;
    CameraSel->addItem("Pinhole", QVariant(1));
    CameraSel->addItem("Pinhole radial 1", QVariant(2));
    CameraSel->addItem("Pinhole radial 3 (default)", QVariant(3));
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
    process_command = new QProcess();
    OutputPath_finished = new QLineEdit(outputpath);
    registerField("Comp_features_OutputPath", OutputPath_finished);
    registerField("Comp_Features_InputPath", InputPath);

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
    advanced_options->addWidget(OutputLabel, 4, 0);
    advanced_options->addWidget(OutputPath, 4, 1);
    advanced_options->addWidget(btnOutputPath, 4, 2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(TerminalMode, 5, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    advanced_options->addWidget(command, 6, 0, 1, 3);
    terminal_fields->addWidget(CommandLabel, 0, 0);  
    terminal_fields->addWidget(btnCancel, 0, 7);
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
    connect(btnCancel,SIGNAL(clicked()),this,SLOT(cancelProcess()));
    connect(AdvancedOptions, &QCheckBox::stateChanged, [this](int box_status) { btnAdvancedOptionsClicked(box_status); } );
    connect(TerminalMode, &QCheckBox::stateChanged, [this](int box_status) { btnTerminalModeClicked(box_status); } );
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    connect(btnOutputPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("outputpath"); });
    connect(btnInputPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("inputpath"); });
    connect(CameraSel,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged), [this](QString new_string) { on_selectors_changed(new_string, "camera_model"); } );
    connect(DescrPres,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged), [this](QString new_string) { on_selectors_changed(new_string, "descr_pres"); } );
    connect(DescrMeth,static_cast<void (QComboBox::*)(const QString &)>(&QComboBox::currentIndexChanged), [this](QString new_string) { on_selectors_changed(new_string, "descr_meth"); } );
    // processes
    connect(process_command, SIGNAL(readyReadStandardOutput()),this, SLOT(rightMessage()) );
    connect(process_command, SIGNAL(readyReadStandardError()), this, SLOT(wrongMessage()) );
}

int Comp_FeaturesPage::nextId() const
{
  return OMVGguiWizard::Page_Pipeline;
}

// When Initializing page (use Show event instead of initialize page) set Text to "Skip" or "Next"
void Comp_FeaturesPage::showEvent(QShowEvent*)
{
    // If Demo dir doesn't exist, download it via git
    if ((QDir(demo_dir).exists() == false) and visited_comp_features == 0)
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
        visited_comp_features = 1;
     }
     else
     {
	check_demo_path();
     }
    // Have we been here before? Set the buttons accordingly
    if(field("Comp_Features_finished").toString() == "false")
    {
	wizard()->button(QWizard::NextButton)->setText(tr("Next >"));
    }
    else 
    {
    	wizard()->button(QWizard::NextButton)->setText(tr("Skip >"));
    }	

    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
}
void Comp_FeaturesPage::cancelProcess()
{
    process_command->terminate();
    txtReport->setTextColor(Qt::red);
    txtReport->append(tr("Process terminated by user"));
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
    btnCancel->setEnabled(false);
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Skip >");
}


void Comp_FeaturesPage::finished_demo_download()
{
    if (QDir(demo_dir).exists() == true)
    {
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (tr(" Download complete!\n"));
	txtReport->moveCursor (QTextCursor::End);
        check_demo_path();	
    } 
}
void Comp_FeaturesPage::failed_demo_download()
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
void Comp_FeaturesPage::check_demo_path()
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
	txtReport->insertPlainText (tr("Arrgh! I failed to delete some extra text files in the /images folder. But you can safely ignore these warnings during feature computing. Have fun playing around."));
	txtReport->moveCursor (QTextCursor::End);
	}
     }
     wizard()->button(QWizard::NextButton)->setEnabled(true);
     btnProcess->setText(tr("Run"));
     btnProcess->setStyleSheet(tr("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;"));
     btnProcess->setEnabled(true);
}

// Event: Click Run
void Comp_FeaturesPage::btnProcessClicked()
{
    // Disable Skip button + Run Button
    wizard()->button(QWizard::NextButton)->setEnabled(false);
    btnProcess->setText(tr("working"));
    btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(false);
    btnCancel->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(true);

    QString str_command;
    txtReport->clear();
    str_command = command->text();
    process_command->start("/bin/bash", QStringList() << "-c" << QString(str_command));
}

// Handle regular output
void Comp_FeaturesPage::rightMessage()
{
    QByteArray strdata = process_command->readAllStandardOutput();
    QString strdata_qstr = strdata;
    QString strdata_qstr_output = strdata;
    // When triggered, get the preview_path output, enable Preview button, don't show
    if (strdata.contains("output_path")) {
	qDebug() << strdata_qstr.replace(QRegExp (".*output_path "), "" );
	qDebug() << strdata_qstr.replace(QRegExp (" end_path.*"), "" );
	OutputPath_finished->setText(strdata_qstr);
	registerField("Comp_features_OutputPath", OutputPath_finished);
	qDebug() << strdata_qstr_output.replace(QRegExp ("output_path.*end_path"), "" );
	// Insert
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
    btnProcess->setText(tr("Finished"));
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(false);
    registerField("Comp_Features_finished", btnProcess);
    }
}

// Handle error messages
void Comp_FeaturesPage::wrongMessage()
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
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(false);
}

// Event: Advanced Options clicked
void Comp_FeaturesPage::btnAdvancedOptionsClicked(int checkstate)
{
    if (checkstate) 
    {
	//Show all the Advanced Options
	command->setEnabled(true);
	TerminalMode->QWidget::show();
	CameraSel->QWidget::show();
	CameraSelLabel->QWidget::show();
	DescrPres->QWidget::show();
	DescrPresLabel->QWidget::show();
	DescrMeth->QWidget::show();
	DescrMethLabel->QWidget::show();
	OutputPath->QWidget::show();
	btnOutputPath->QWidget::show();
	OutputLabel->QWidget::show();
    }
    else 
    {
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
	OutputPath->QWidget::hide();
	btnOutputPath->QWidget::hide();
	OutputLabel->QWidget::hide();
    }
}

// Event: Terminal Mode clicked
void Comp_FeaturesPage::btnTerminalModeClicked(int checkstate)
{
    if (checkstate) 
    {
	command->setEnabled(true);
	command->QWidget::show();
    }
    else 
    {
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: paths changed
void Comp_FeaturesPage::btnPathbuttonsClicked(QString mode)
{
    QString str_get_commando;
    QString Folder;
    QString selection_descr;

    if (mode == "inputpath") { str_get_commando = InputPath->text(); selection_descr = tr("Choose folder containing your sfmdata.json file"); }
    else if (mode == "outputpath") { str_get_commando = OutputPath->text(); selection_descr = tr("Choose output folder"); }

    // launch selection menu
    Folder = QFileDialog::getExistingDirectory(this, selection_descr, str_get_commando, QFileDialog::ShowDirsOnly |QFileDialog::DontUseNativeDialog);

    QString str_commando;
    str_commando = command->text();
    // change both parts if inputpath
    qDebug() << str_commando.replace(QRegExp (mode + "=\"([^\"]*)\""), mode + "=\"" + Folder + "/\"");
    if (mode == "inputpath") 
    {  
	qDebug() << str_commando.replace(QRegExp ("outputpath=\"([^\"]*)\""), "outputpath=\"" + Folder + "_out/matches/\"");
    }
    command->setText(str_commando);

    if (mode == "inputpath") { InputPath->setText(Folder + "/"); OutputPath->setText(Folder + "_out/matches/");}
    else if (mode == "outputpath") { OutputPath->setText(Folder + "/"); }

    // Have we been here before? Enable re-running
    enable_run_again();
}

// Event: Field command changed --> Enable run (Assume, whoever clicks that knows what he's doing)
void Comp_FeaturesPage::fldcommandClicked()
{
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
}

// Event: Selector changed
void Comp_FeaturesPage::on_selectors_changed(QString selection_string, QString option_decl)
{
    
    if (selection_string == "Pinhole") { selection_string = "1"; }
    if (selection_string == "Pinhole radial 1") { selection_string = "2"; }
    if (selection_string == "Pinhole radial 3 (default)") { selection_string = "3"; }

    QString str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp (option_decl + "=\"([^\"]*)\""), option_decl + "=\"" + selection_string + "\"");
    command->setText(str_commando);

    enable_run_again();
}

// Have we been here before? Enable running again
void Comp_FeaturesPage::enable_run_again()
{
    if(field("Comp_Features_finished").toString() == "false") {
	btnProcess->setText(tr("Run"));
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
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
    OptionsPipeline = new QLineEdit("");
    OptionsPipeline->QWidget::hide();
    btnInputPath = new QPushButton(tr("Select"));
    btnInputPath->QWidget::hide();
    InputLabel = new QLabel(tr("Matches Path:"));
    InputLabel->QWidget::hide();
    OutputPath = new QLineEdit(outputpath);
    btnOutputPath = new QPushButton(tr("Select"));
    OutputLabel = new QLabel(tr("Output folder:"));
    OutputPath->QWidget::hide();
    btnOutputPath->QWidget::hide();
    OutputLabel->QWidget::hide();
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
    MatrixSel->addItem(tr("Essential matrix filtering (Standard for GlobalSfM)"), QVariant(1));
    MatrixSel->addItem(tr("Fundamental matrix filtering (Standard for IncrementalSfM)"), QVariant(2));
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
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnCancel = new QPushButton(tr("Cancel"));
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(false);
    TerminalMode = new QCheckBox(tr("Terminal Mode (Advanced Users)"));
    TerminalMode->QWidget::hide();
    CommandLabel = new QLabel(tr("Output:"));

    // Set up main Layout
    main_grid = new QGridLayout;  
    input_fields = new QGridLayout;  
    image_selector_grid = new QGridLayout;  
    terminal_fields = new QGridLayout;     
    advanced_options = new QGridLayout;   

    // Register fields of vars to use elsewhere.. (don't use an asterisk to not make it mandatory)
    OutputPath_Pipeline = new QLineEdit("");
    OutputPath_Pipeline->QWidget::hide();
    registerField(tr("Pipeline_OutputPath"), OutputPath_Pipeline);
    StatusPipelinePage = new QLineEdit("init");
    registerField("PipelinePage_status", StatusPipelinePage);
    preview_pipeline = new QLineEdit;
    registerField("Preview_Pipeline", preview_pipeline);

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
    advanced_options->addWidget(MatrixSelLabel, 1, 0);
    advanced_options->addWidget(MatrixSel, 1, 1);
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
    advanced_options->addWidget(OutputLabel, 6, 0);
    advanced_options->addWidget(OutputPath, 6, 1);
    advanced_options->addWidget(btnOutputPath, 6, 2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(TerminalMode, 7, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    advanced_options->addWidget(command, 8, 0, 1, 3);

    terminal_fields->addWidget(CommandLabel, 0, 0); 
    terminal_fields->addWidget(btnCancel, 0, 7); 
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

    // Initialize process
    process_command = new QProcess();

    // Connect buttons with processes
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(btnCancel,SIGNAL(clicked()),this,SLOT(cancelProcess()));
    connect(AdvancedOptions, &QCheckBox::stateChanged, [this](int box_status) { btnAdvancedOptionsClicked(box_status); } );
    connect(TerminalMode, &QCheckBox::stateChanged, [this](int box_status) { btnTerminalModeClicked(box_status); } );
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    // Paths
    connect(btnInputPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("inputpath"); });
    connect(btnOutputPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("outputpath"); });
    connect(btnImagesFolderPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("imagespath"); });
    connect(solverImage1Button, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("image1"); });
    connect(solverImage2Button, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("image2"); });
    // various
    connect(sliderRatio, SIGNAL(valueChanged(int)),this, SLOT(setRatio(int)));
    // selectors
    connect(CameraSel,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this](int selection_num) { on_selectors_changed(selection_num, "camera_model"); } );
    connect(MatrixSel,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this](int selection_num) { on_selectors_changed(selection_num, "matrix_filter"); } );
    connect(PipelineSel,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this](int selection_num) { on_selectors_changed(selection_num, "solver"); } );
    // Processes
    connect(process_command, SIGNAL(readyReadStandardOutput()),this, SLOT(rightMessage()) );
    connect(process_command, SIGNAL(readyReadStandardError()), this, SLOT(wrongMessage()) );
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
    // Set paths according to the path status from the previous page
    QString input_dir_path = field("Comp_Features_InputPath").toString();
    QString output_dir = field("Comp_features_OutputPath").toString();
    QString output_dir_cut = field("Comp_features_OutputPath").toString().mid(0, field("Comp_features_OutputPath").toString().length()-1);
    QString mvs_dir = output_dir_cut.mid(0, output_dir_cut.lastIndexOf("/")) + "/reconstruction_global/";

    QString str_commando;
    str_commando = command->text();
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + output_dir + "\"");
    qDebug() << str_commando.replace(QRegExp ("matchespath=\"([^\"]*)\""), "matchespath=\"" + output_dir + "\"");
    qDebug() << str_commando.replace(QRegExp ("imagespath=\"([^\"]*)\""), "imagespath=\"" + input_dir_path + "\"");
    qDebug() << str_commando.replace(QRegExp ("outputpath=\"([^\"]*)\""), "outputpath=\"" + mvs_dir + "\"");
    command->setText(str_commando);

    // Have we been here before? Set the paths accordingly
    if (field("PipelinePage_status").toString() == "init") 
    {
	InputPath->setText(output_dir);
	OutputPath->setText(mvs_dir);
	OutputPath_Pipeline->setText(mvs_dir);
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
    else
    {
	wizard()->button(QWizard::CustomButton1)->setEnabled(false);	
    }
    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
}
void PipelinePage::cancelProcess()
{
    process_command->terminate();
    txtReport->setTextColor(Qt::red);
    txtReport->append(tr("Process terminated by user"));
    wizard()->button(QWizard::BackButton)->setEnabled(true);
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
    btnCancel->setEnabled(false);
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    wizard()->button(QWizard::NextButton)->setEnabled(true);
    wizard()->button(QWizard::NextButton)->setText("Skip >");
}

// Event: get paths
void PipelinePage::btnPathbuttonsClicked(QString mode)
{
    QString str_get_commando;
    QString path;
    QString selection_descr;
    QString search_dir;

    // Event: Select Images
    if ((mode == "image1") or (mode == "image2")) 
    { 
	QString file = QFileDialog::getOpenFileName(this, tr("Choose an image to initialize matching"), field("Comp_Features_InputPath").toString(), selfilter_images);
        QFileInfo filepath(file); 
        path = filepath.fileName();
    }
    // Event: Select folders
    else 
    { 
	if (mode == "imagespath") { selection_descr = tr("Choose folder containing your input images"); search_dir = ImagesFolderPath->text(); }
	if (mode == "inputpath") { selection_descr = tr("Choose matching folder (contains the JSON file)"); search_dir = InputPath->text(); }
	if (mode == "outputpath") { selection_descr = tr("Choose folder to export the matches files to"); search_dir = OutputPath->text(); }

	path = QFileDialog::getExistingDirectory(this, selection_descr, search_dir, QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog) + "/";
    }

    if (mode == "image1") { solverImage1->setText(path); }
    if (mode == "image2") { solverImage2->setText(path); }
    if (mode == "imagespath") { ImagesFolderPath->setText(path); }
    if (mode == "inputpath") { InputPath->setText(path); }
    if (mode == "outputpath") { OutputPath->setText(path); }

    // replace vars
    QString str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp (mode + "=\"([^\"]*)\""), mode + "=\"" + path + "\"");
    // change matchespath as well, when inputpath
    if (mode == "inputpath") { mode = "matchespath";  qDebug() << str_commando.replace(QRegExp (mode + "=\"([^\"]*)\""), mode + "=\"" + path + "\""); }
    command->setText(str_commando);

    // Have we been here before? Enable re-running
    enable_rerunning();
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
	btnCancel->setEnabled(true);
 	btnCancel->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	wizard()->button(QWizard::CustomButton1)->setEnabled(false);	

	QString str_command;
	txtReport->clear();
	str_command = command->text();
	process_command->start("/bin/bash", QStringList() << "-c" << QString(str_command));
    }
}

// Handle regular output
void PipelinePage::rightMessage()
{
    QByteArray strdata = process_command->readAllStandardOutput();
    QString strdata_qstr = strdata;
    QString strdata_qstr_copy = strdata;
    QString strdata_qstr_options = strdata;
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
	OutputPath_Pipeline->setText(strdata_qstr_copy);
    	registerField("Pipeline_Output", OutputPath_Pipeline);
	// Also get options
	qDebug() << strdata_qstr_options.replace(QRegExp (".*options_used "), "" );
	qDebug() << strdata_qstr_options.replace(QRegExp (" end_options_used.*"), "" );
	OptionsPipeline->setText(strdata_qstr_options);
    	registerField("Pipeline_Options", OptionsPipeline);
	// Insert
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (strdata_qstr_output);
	txtReport->moveCursor (QTextCursor::End);
     	// Run Preview 
	launchPreview(strdata_qstr, " (Step: Pipeline)",strdata_qstr_options);
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
	btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
	btnCancel->setEnabled(false);
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
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(false);
}

// Event: Advanced Options clicked
void PipelinePage::btnAdvancedOptionsClicked(int checkstate)
{
    if (checkstate) 
    {
	//Show all the Advanced Options
	command->setEnabled(true);
	TerminalMode->QWidget::show();
	InputLabel->QWidget::show();
	InputPath->QWidget::show();
	btnInputPath->QWidget::show();
        sliderRatio->QWidget::show();
   	ratioValue->QWidget::show();
	ratioLabel->QWidget::show();
        MatrixSel->QWidget::show();
	MatrixSelLabel->QWidget::show();
	OutputPath->QWidget::show();
	btnOutputPath->QWidget::show();
	OutputLabel->QWidget::show();

	// If Pipeline = Incremental show image folder selection
        if(PipelineSel->itemData(PipelineSel->currentIndex()).toString() == "1")
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
	OutputPath->QWidget::hide();
	btnOutputPath->QWidget::hide();
	OutputLabel->QWidget::hide();
	TerminalMode->QWidget::hide();
	TerminalMode->QCheckBox::setChecked(false);
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: Terminal Mode clicked
void PipelinePage::btnTerminalModeClicked(int checkstate)
{
    if (checkstate)
    {
	command->setEnabled(true);
	command->QWidget::show();
    }
    else 
    {
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

// Event: Selector changed
void PipelinePage::on_selectors_changed(int selection_num, QString option_decl)
{
    QString str_commando = command->text();
    QString replace_char;

    // Event: Pipeline Selector changed
    if (option_decl == "solver")
    {  
	replace_char = QString::number(selection_num);
	// Dirty: hide non-affected options, show affected for Global SfM
	if(selection_num == 1)
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
	    // Change matrix selector according to Pipeline (Incremental: g, Global: e)
	    MatrixSel->setCurrentIndex(2);
	    qDebug() << str_commando.replace(QRegExp ("matrix_filter=\"([^\"]*)\""), "matrix_filter=\"e\"");
	}

	// Dirty: show non-affected options for Incremental SfM
	else if(selection_num == 0)
	{
	    // Hide Matrix selector + show Images
	    if(AdvancedOptions->checkState() == Qt::Checked)
	    {
		MatrixSel->QWidget::show();
		MatrixSelLabel->QWidget::show();
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
	    // Change matrix selector according to Pipeline (Incremental: g, Global: e)
            MatrixSel->setCurrentIndex(1);
	    qDebug() << str_commando.replace(QRegExp ("matrix_filter=\"([^\"]*)\""), "matrix_filter=\"f\"");
	}
    }
    // Event: Pipeline Selector changed
    else if (option_decl == "matrix_filter")
    {
	if(selection_num == 0) { replace_char = "e"; }
	else if(selection_num == 1) { replace_char = "f"; }
	else if(selection_num == 2) { replace_char = "h"; }
    }
    // Event: Camera changed
    else if (option_decl == "camera_model") { replace_char = QString::number(selection_num+1); }

    // general replace
    qDebug() << str_commando.replace(QRegExp (option_decl + "=\"([^\"]*)\""), option_decl + "=\"" + replace_char + "\"");
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
    command = new QLineEdit(initialcommandline_mvs_openMVS);
    command->setStyleSheet("color: #ffffff; background-color: #300a24;");
    command->setEnabled(false);
    command->QWidget::hide();
    InputPath = new QLineEdit(work_dir);
    InputPath->QWidget::hide();
    btnInputPath = new QPushButton(tr("Select"));
    btnInputPath->QWidget::hide();
    InputLabel = new QLabel(tr("sfm_data.json Folder:"));
    InputLabel->QWidget::hide();
    OptionsMVS = new QLineEdit("");
    OptionsMVS->QWidget::hide();
    OutputPath = new QLineEdit(work_dir);
    OutputPath->QWidget::hide();
    btnOutputPath = new QPushButton(tr("Select"));
    btnOutputPath->QWidget::hide();
    OutputLabel = new QLabel(tr("Output Folder:"));
    OutputLabel->QWidget::hide();
    TerminalMode = new QCheckBox(tr("Terminal Mode (Expert Users)"));
    TerminalMode->QWidget::hide();
    btnProcess = new QPushButton(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnCancel = new QPushButton(tr("Cancel"));
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(false);
    CommandLabel = new QLabel(tr("Output:"));
    // openMVS specific
    UseDensify = new QCheckBox(tr("[openmVS] Densify Point Cloud"));
    UseDensify->QWidget::hide();
    UseDensify->QAbstractButton::setChecked(true);
    UseRefine = new QCheckBox(tr("[openmVS] Refine Mesh"));
    UseRefine->QWidget::hide();
    UseRefine->QAbstractButton::setChecked(true);
    // CMVS specific
    // Documentation for PMVS2: http://www.di.ens.fr/pmvs/documentation.html
    ImageCountLabel = new QLabel(tr("[Bundler] Max imagecount per cluster:"));
    ImageCountLabel->QWidget::hide();
    ImageCount = new QLineEdit("100");
    ImageCount->QWidget::setFixedWidth(100);
    ImageCount->setAlignment(Qt::AlignHCenter);	
    ImageCount->QWidget::hide();
    numCPULabel = new QLabel(tr("Number of CPUs to use (incl. virtual):"));
    numCPULabel->QWidget::hide();
    numCPU = new QLineEdit(get_num_of_CPUs());
    numCPU->QWidget::setFixedWidth(100);
    numCPU->setAlignment(Qt::AlignHCenter);	
    numCPU->QWidget::hide();
    LevelLabel = new QLabel(tr("Level (Input Image shrinking - 0 = 100 % | 1 = 50 %):"));
    LevelLabel->QWidget::hide();
    level = new QLineEdit("1");
    level->QWidget::setFixedWidth(100);
    level->setAlignment(Qt::AlignHCenter);	
    level->QWidget::hide();
    csizeLabel = new QLabel(tr("Cell size (bigger cell size = sparser reconstruction):"));
    csizeLabel->QWidget::hide();
    csize = new QLineEdit("2");
    csize->QWidget::setFixedWidth(100);
    csize->setAlignment(Qt::AlignHCenter);	
    csize->QWidget::hide();
    thresholdLabel = new QLabel(tr("Consistency threshold (-1 (bad) - +1 (very good)):"));
    thresholdLabel->QWidget::hide();
    threshold = new QLineEdit("0.7");
    threshold->QWidget::setFixedWidth(100);
    threshold->setAlignment(Qt::AlignHCenter);	
    threshold->QWidget::hide();
    wsizeLabel = new QLabel(tr("Width for field color sampling (bigger numbers = better results):"));
    wsizeLabel->QWidget::hide();
    wsize = new QLineEdit("7");
    wsize->QWidget::setFixedWidth(100);
    wsize->setAlignment(Qt::AlignHCenter);	
    wsize->QWidget::hide();
    minImageLabel = new QLabel(tr("Minimum number of images, for each 3d point to be reconstructed:"));
    minImageLabel->QWidget::hide();
    minImage = new QLineEdit("3");
    minImage->QWidget::setFixedWidth(100);
    minImage->setAlignment(Qt::AlignHCenter);	
    minImage->QWidget::hide();
    // Layout
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
    advanced_options->addWidget(InputLabel, 2, 0);
    advanced_options->addWidget(InputPath, 2, 1);
    advanced_options->addWidget(btnInputPath, 2, 2);
    // openMVS options
    UseDensify->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(UseDensify, 3, 0, 1, 2);
    UseRefine->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(UseRefine, 4, 0, 1, 2);
    // PMVS options
    ImageCountLabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(ImageCountLabel, 5, 0, 1, 2);
    advanced_options->addWidget(ImageCount, 5 , 2);
    numCPULabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(numCPULabel, 6, 0, 1, 2);
    advanced_options->addWidget(numCPU, 6 , 2);
    LevelLabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(LevelLabel, 7, 0, 1, 2);
    advanced_options->addWidget(level, 7 , 2);
    csizeLabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(csizeLabel, 8, 0, 1, 2);
    advanced_options->addWidget(csize, 8, 2);
    thresholdLabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(thresholdLabel, 9, 0, 1, 2);
    advanced_options->addWidget(threshold, 9, 2);
    wsizeLabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(wsizeLabel, 10, 0, 1, 2);
    advanced_options->addWidget(wsize, 10, 2);
    minImageLabel->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(minImageLabel, 11, 0, 1, 2);
    advanced_options->addWidget(minImage, 11, 2);
    // Shared options
    advanced_options->addWidget(OutputLabel, 12, 0);
    advanced_options->addWidget(OutputPath, 12, 1);
    advanced_options->addWidget(btnOutputPath, 12, 2);
    TerminalMode->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    advanced_options->addWidget(TerminalMode, 13, 0, 1, 2);
    command->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);	
    advanced_options->addWidget(command, 14, 0, 1, 3);
    terminal_fields->addWidget(CommandLabel, 0, 0);  
    terminal_fields->addWidget(btnCancel, 0, 7);
    terminal_fields->addWidget(btnProcess, 0, 8);
    txtReport->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    terminal_fields->addWidget(txtReport, 1, 0, 1, 9);

    // Insert into main Layout
    main_grid->addLayout(input_fields, 0, 0);
    main_grid->addLayout(advanced_options, 2, 0);
    main_grid->addLayout(terminal_fields, 3, 0);

    // Finalize
    setLayout(main_grid);
  
   // Register fields
    process_command = new QProcess();
    StatusMVSSelectorPage = new QLineEdit("init");
    registerField("MVSSelectorPage_status", StatusMVSSelectorPage);

    // connect general options
    connect(btnCancel,SIGNAL(clicked()),this,SLOT(cancelProcess()));
    connect(btnProcess,SIGNAL(clicked()),this,SLOT(btnProcessClicked()));
    connect(command,SIGNAL(textEdited(QString)),this,SLOT(fldcommandClicked()));
    connect(btnInputPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("inputpath"); });
    connect(btnOutputPath, &QPushButton::clicked, [this]() { btnPathbuttonsClicked("output_dir"); });
    connect(AdvancedOptions, &QCheckBox::stateChanged, [this](int box_status) { MVSSelectorPage::btnAdvancedOptionsClicked(box_status); } );
    connect(TerminalMode, &QCheckBox::stateChanged, [this](int box_status) { MVSSelectorPage::btnTerminalModeClicked(box_status); } );
    connect(MVSSel,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [this](int selection_num) { MVSSelectorPage::on_MVSSel_changed(selection_num); } );
    // connect openMVS options
    connect(UseDensify, &QCheckBox::stateChanged, [this](int box_status) { MVSSelectorPage::btnUseopenMVSoptionsClicked(box_status, "use_densify"); } );
    connect(UseRefine, &QCheckBox::stateChanged, [this](int box_status) { MVSSelectorPage::btnUseopenMVSoptionsClicked(box_status, "use_refine"); } );
    // connect PMVS options
    connect(ImageCount, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "max_imagecount"); } );
    connect(numCPU, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "cpu"); } );
    connect(level, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "level"); } );
    connect(csize, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "csize"); } );
    connect(threshold, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "threshold"); } );
    connect(wsize, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "wsize"); } );
    connect(minImage, &QLineEdit::textEdited, [this](QString new_content) { MVSSelectorPage::pmvsOptionsclicked(new_content, "minImageNum"); } );
    // connect output
    connect(process_command, SIGNAL(readyReadStandardOutput()),this, SLOT(rightMessage()) );
    connect(process_command, SIGNAL(readyReadStandardError()), this, SLOT(wrongMessage()) );
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
    if(field("MVSSelectorPage_status").toString() == "finished") 
    {
    	wizard()->button(QWizard::FinishButton)->setEnabled(true);
    }
    else
    {
    	wizard()->button(QWizard::FinishButton)->setEnabled(false);
    }
    get_standard_paths(command->text());

    // Do we want the preview button here? Naaahhh....
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);
    // Is there a Preview path already? Enable Preview button
    if (field("Preview_MVS").toString() != "")
    {
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);	
    }
    else
    {
	wizard()->button(QWizard::CustomButton1)->setEnabled(false);	
    }
}
void MVSSelectorPage::cancelProcess()
{
    process_command->terminate();
    txtReport->setTextColor(Qt::red);
    txtReport->append(tr("Process terminated by user"));
    wizard()->button(QWizard::BackButton)->setEnabled(true);
    btnProcess->setText(tr("Run"));
    btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(true);
    btnCancel->setEnabled(false);
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
}

// Get standard paths when changing menu entries
void MVSSelectorPage::get_standard_paths(QString str_commando)
{
    // set inputpath + outputpath accordingly
    QString str(str_commando); 
    qDebug() << str_commando.replace(QRegExp ("inputpath=\"([^\"]*)\""), "inputpath=\"" + field("Pipeline_OutputPath").toString() + "\"");
    QString input_path_cut = field("Pipeline_OutputPath").toString().mid(0,  field("Pipeline_OutputPath").toString().length()-1);
    QString output_dir = input_path_cut.mid(0, input_path_cut.lastIndexOf("/")) + "/MVS_out/";
    qDebug() << str_commando.replace(QRegExp ("output_dir=\"([^\"]*)\""), "output_dir=\"" + output_dir + "\"");
    command->setText(str_commando);

    InputPath->setText(field("Pipeline_OutputPath").toString());
    OutputPath->setText(output_dir);
}

// Enable re-running
void MVSSelectorPage::enable_rerunning()
{
    if (field("MVSSelectorPage_status").toString() == "finished") 
    {
	btnProcess->setText(tr("Run"));
	btnProcess->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
	btnProcess->setEnabled(true);
    }
}

// Event: path
void MVSSelectorPage::btnPathbuttonsClicked(QString mode)
{
    QString str_get_commando;
    QString Folder;
    QString selection_descr;

    if (mode == "inputpath") { str_get_commando = InputPath->text(); selection_descr = tr("Choose folder containing your sfmdata.json file"); }
    else if (mode == "output_dir") { str_get_commando = OutputPath->text(); selection_descr = tr("Choose output folder"); }

    // launch selection menu
    Folder = QFileDialog::getExistingDirectory(this, selection_descr, str_get_commando, QFileDialog::ShowDirsOnly |QFileDialog::DontUseNativeDialog) + "/";

    if (mode == "inputpath") { InputPath->setText(Folder); }
    else if (mode == "output_dir") { OutputPath->setText(Folder); }

    QString str_commando;
    str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp (mode + "=\"([^\"]*)\""), mode + "=\"" + Folder + "\"");
    command->setText(str_commando);

    // Have we been here before? Enable re-running
    enable_rerunning();
}


// Event: Click Run
void MVSSelectorPage::btnProcessClicked()
{
    // Disable Skip button + Run Button + Preview button
    wizard()->button(QWizard::FinishButton)->setEnabled(false);
    wizard()->button(QWizard::BackButton)->setEnabled(false);
    btnCancel->setStyleSheet("border:2px solid #f07b4c; background-color: #300a24; color: #ffffff;");
    btnCancel->setEnabled(true);
    btnProcess->setText(tr("working"));
    btnProcess->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
    btnProcess->setEnabled(false);
    wizard()->button(QWizard::CustomButton1)->setEnabled(false);

    QString str_command;
    txtReport->clear();
    str_command = command->text();

    process_command->start("/bin/bash", QStringList() << "-c" << QString(str_command));
}

// Handle regular output
void MVSSelectorPage::rightMessage()
{
    QByteArray strdata = process_command->readAllStandardOutput();
    QString strdata_qstr = strdata;
    QString strdata_qstr_output = strdata;
    QString strdata_qstr_options = strdata;
    if (strdata.contains("preview_path")) 
    {
	qDebug() << strdata_qstr.replace(QRegExp (".*preview_path "), "" );
	qDebug() << strdata_qstr.replace(QRegExp (" end_path.*"), "" );
	preview_mvs->setText(strdata_qstr);
	registerField("Preview_MVS", preview_mvs);
	wizard()->button(QWizard::CustomButton1)->setEnabled(true);
	qDebug() << strdata_qstr_output.replace(QRegExp ("preview_path.*end_options_used"), "" );
	// Also get options
	qDebug() << strdata_qstr_options.replace(QRegExp (".*options_used "), "" );
	qDebug() << strdata_qstr_options.replace(QRegExp (" end_options_used.*"), "" );
	OptionsMVS->setText(strdata_qstr_options);
    	registerField("MVS_Options", OptionsMVS);
	// Print output
	txtReport->moveCursor (QTextCursor::End);
	txtReport->insertPlainText (strdata_qstr_output);
	txtReport->moveCursor (QTextCursor::End);
     	// Run Preview 
    	launchPreview(strdata_qstr, " (Step: MVS)", strdata_qstr_options);
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
	btnProcess->setText(tr("Finished"));
   	StatusMVSSelectorPage->setText("finished");
	registerField("MVSSelectorPage_status", StatusMVSSelectorPage);
        btnCancel->setEnabled(false);
        btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
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
    btnCancel->setEnabled(false);
    btnCancel->setStyleSheet("border:2px solid #aaaaaa; background-color: #300a24; color: #ffffff;");
}

// Event: Advanced Options clicked
void MVSSelectorPage::btnAdvancedOptionsClicked(int checkstate)
{
    QString get_SelItem = MVSSel->itemData(MVSSel->currentIndex()).toString();

    if (checkstate) {
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
	if (get_SelItem == "1")
        {
   	    UseDensify->QWidget::show();
   	    UseRefine->QWidget::show();
        }
        else if (get_SelItem == "3")
        {
	    PMVSoptionsdisplay();
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
	PMVSoptionshide();
    }
}

// Event: Terminal Mode clicked
void MVSSelectorPage::btnTerminalModeClicked(int checkstate)
{
    if (checkstate) {
	command->setEnabled(true);
	command->QWidget::show();
    }
    else {
	command->setEnabled(false);
	command->QWidget::hide();
    }
}

// Event: Set PMVS options visibility
void MVSSelectorPage::PMVSoptionsdisplay()
{
    ImageCountLabel->QWidget::show();
    ImageCount->QWidget::show();
    numCPULabel->QWidget::show();
    numCPU->QWidget::show();
    LevelLabel->QWidget::show();
    level->QWidget::show();
    csizeLabel->QWidget::show();
    csize->QWidget::show();
    thresholdLabel->QWidget::show();
    threshold->QWidget::show();
    wsizeLabel->QWidget::show();
    wsize->QWidget::show();
    minImageLabel->QWidget::show();
    minImage->QWidget::show();
}
void MVSSelectorPage::PMVSoptionshide()
{
    ImageCountLabel->QWidget::hide();
    ImageCount->QWidget::hide();
    numCPULabel->QWidget::hide();
    numCPU->QWidget::hide();
    LevelLabel->QWidget::hide();
    level->QWidget::hide();
    csizeLabel->QWidget::hide();
    csize->QWidget::hide();
    thresholdLabel->QWidget::hide();
    threshold->QWidget::hide();
    wsizeLabel->QWidget::hide();
    wsize->QWidget::hide();
    minImageLabel->QWidget::hide();
    minImage->QWidget::hide();
    // if we are not in CMVS meun, reset values to standard
    if (MVSSel->itemData(MVSSel->currentIndex()).toString() != "3")
    {
	ImageCount->setText("100");
	numCPU->setText(get_num_of_CPUs());
	level->setText("1");
	csize->setText("2");
	threshold->setText("0.7");
	wsize->setText("7");
	minImage->setText("3");
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
void MVSSelectorPage::on_MVSSel_changed(int selection_num)
{
    QString str_commando;

    if(selection_num == 0)
    {
	// Hide Matrix selector + show Images
	if(AdvancedOptions->checkState() == Qt::Checked)
	{
   	    UseDensify->QWidget::show();
   	    UseRefine->QWidget::show();
	}
        str_commando = initialcommandline_mvs_openMVS;
	UseDensify->QAbstractButton::setChecked(true);
	UseRefine->QAbstractButton::setChecked(true);
	PMVSoptionshide();
    }
    else if(selection_num == 1)
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();

        str_commando = initialcommandline_mvs_stand;
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"pmvs\"");
	PMVSoptionshide();
    }
    else if(selection_num == 2)
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();

        str_commando = initialcommandline_mvs_CMVS;
	qDebug() << str_commando.replace(QRegExp ("cpu=\"([^\"]*)\""), "cpu=\"" + get_num_of_CPUs() + "\"");

	if(AdvancedOptions->checkState() == Qt::Checked)
	{
	PMVSoptionsdisplay();
	}
    }
    else if(selection_num == 3)
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();

        str_commando = initialcommandline_mvs_stand;
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"cmpmvs\"");
	PMVSoptionshide();
    }
    else if(selection_num == 4)
    {
	UseDensify->QWidget::hide();
	UseRefine->QWidget::hide();

        str_commando = initialcommandline_mvs_stand;
	qDebug() << str_commando.replace(QRegExp ("step=\"([^\"]*)\""), "step=\"mve\"");
	PMVSoptionshide();
    }
    enable_rerunning();
    get_standard_paths(str_commando);
}

// Event openMVS Clicked
void MVSSelectorPage::btnUseopenMVSoptionsClicked(int checkstate, QString option_decl)
{
    QString str_commando;
    str_commando = command->text();
    if(checkstate)
    {
    qDebug() << str_commando.replace(QRegExp (option_decl + "=\"([^\"]*)\""), option_decl + "=\"ON\"");
    }
    else
    {
    qDebug() << str_commando.replace(QRegExp (option_decl + "=\"([^\"]*)\""), option_decl + "=\"OFF\"");
    }
    command->setText(str_commando);
}


// Event pmvsOptions clicked
void MVSSelectorPage::pmvsOptionsclicked(QString new_content, QString option_decl)
{
    QString str_commando;
    str_commando = command->text();
    qDebug() << str_commando.replace(QRegExp (option_decl + "=\"([^\"]*)\""), option_decl + "=\"" + new_content + "\"");
    command->setText(str_commando);
}
