#include <QApplication>
#include <QTranslator>
#include <QLocale>
#include <QLibraryInfo>

#include "software/SfM/SfM_gui.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

#ifndef QT_NO_TRANSLATION
    QString translatorFileName = QLatin1String("qt_");
    translatorFileName += QLocale::system().name();
    QTranslator *translator = new QTranslator(&app);
    if (translator->load(translatorFileName, QLibraryInfo::location(QLibraryInfo::TranslationsPath)))
        app.installTranslator(translator);
#endif

    OMVGguiWizard wizard;
    wizard.show();
    return app.exec();
}
