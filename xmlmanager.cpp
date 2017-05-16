#include "xmlmanager.h"

XMLManager::XMLManager()
{

    this->xmlWritePath = "dump.xml";
    this->xmlInterpolatedWritePath = "dump_interpolate.xml";
}

XMLManager::~XMLManager()
{
    delete this->doc;
}

void XMLManager::writeClickedObjects(std::vector<RealObject *> objects)
{
    this->doc = new QDomDocument();
    QDomElement trackletsElement = this->doc->createElement("tracklets");
    this->doc->appendChild(trackletsElement);

    QDomElement countElement = this->doc->createElement("count");
    countElement.appendChild(this->doc->createTextNode(QString::number(objects.size())));
    trackletsElement.appendChild(countElement);

    QDomElement itemVersionElement = this->doc->createElement("item_version");
    itemVersionElement.appendChild(this->doc->createTextNode(QString::number(1)));
    trackletsElement.appendChild(itemVersionElement);

    for(int i = 0; i<objects.size(); i++){
        RealObject *robj = objects.at(i);
        /*
        //v1 : compute mean size
        int nbFramesClicked = 0;
        float meanWidth = 0;
        float meanHeight = 0;
        float meanLength = 0;

        //std::map<int, Pose3D> annotations3D
        std::map<int, Pose3D>::iterator it;
        for(it = robj->annotations3D.begin(); it != robj->annotations3D.end(); ++it){
            std::cout << "frame : " << it->first << std::endl;
            Pose3D pose3D = it->second;
            meanWidth += pose3D.width;
            meanHeight += pose3D.height;
            meanLength += pose3D.length;

            nbFramesClicked++;
        }
        meanWidth /=nbFramesClicked;
        meanHeight /=nbFramesClicked;
        meanLength /=nbFramesClicked;

        std::cout << QString("meanWidth : %1, meanHeight : %2, meanLength : %3").arg(meanWidth).arg(meanHeight).arg(meanLength).toStdString() << std::endl;
        */

        //item (object) add attribite
        QDomElement itemElement = this->doc->createElement("item");
        //id(ajout perso)
        itemElement.setAttribute("id", robj->id);

        //objectType
        QDomElement objectTypeElement = this->doc->createElement("objectType");
        objectTypeElement.appendChild(this->doc->createTextNode(objectTypeToString(robj->objectType)));
        itemElement.appendChild(objectTypeElement);

        //h
        QDomElement hElement = this->doc->createElement("h");
        hElement.appendChild(this->doc->createTextNode(QString::number(robj->height)));
        itemElement.appendChild(hElement);
        //w
        QDomElement wElement = this->doc->createElement("w");
        wElement.appendChild(this->doc->createTextNode(QString::number(robj->width)));
        itemElement.appendChild(wElement);
        //l
        QDomElement lElement = this->doc->createElement("l");
        lElement.appendChild(this->doc->createTextNode(QString::number(robj->length)));
        itemElement.appendChild(lElement);


        //first_frame
        std::map<int, Pose3D>::iterator it = robj->annotations3D.begin();
        int firstFrame = it->first;
        QDomElement firstFrameElement = this->doc->createElement("first_frame");
        firstFrameElement.appendChild(this->doc->createTextNode(QString::number(firstFrame)));
        itemElement.appendChild(firstFrameElement);

        //poses
        QDomElement posesElement = this->doc->createElement("poses");
        itemElement.appendChild(posesElement);



        //count (poses)

        //item_version (poses)

        for(it = robj->annotations3D.begin(); it != robj->annotations3D.end(); ++it){
            Pose3D pose = it->second;
            int currentFrame = it->first;

            //item
            QDomElement posesItemElement = this->doc->createElement("item");
            posesElement.appendChild(posesItemElement);

            QDomElement frameElement = this->doc->createElement("frame");
            frameElement.appendChild(this->doc->createTextNode(QString::number(currentFrame)));
            posesItemElement.appendChild(frameElement);

            //tx
            QDomElement txElement = this->doc->createElement("tx");
            txElement.appendChild(this->doc->createTextNode(QString::number(pose.x)));
            posesItemElement.appendChild(txElement);
            //ty
            QDomElement tyElement = this->doc->createElement("ty");
            tyElement.appendChild(this->doc->createTextNode(QString::number(pose.y)));
            posesItemElement.appendChild(tyElement);
            //tz
            QDomElement tzElement = this->doc->createElement("tz");
            tzElement.appendChild(this->doc->createTextNode(QString::number(pose.z)));
            posesItemElement.appendChild(tzElement);

            //ROTATIONS RANDOM !!!!!!!!!!!!!
            //rx
            QDomElement rxElement = this->doc->createElement("rx");
            rxElement.appendChild(this->doc->createTextNode(QString::number(pose.phi)));
            posesItemElement.appendChild(rxElement);
            //ry
            QDomElement ryElement = this->doc->createElement("ry");
            ryElement.appendChild(this->doc->createTextNode(QString::number(pose.psi)));
            posesItemElement.appendChild(ryElement);
            //rz
            QDomElement rzElement = this->doc->createElement("rz");
            rzElement.appendChild(this->doc->createTextNode(QString::number(pose.theta)));
            posesItemElement.appendChild(rzElement);

            //OCCULTATIONS A FAIRE !!!!!!!!!!!!!!!

        }

        trackletsElement.appendChild(itemElement);
    }



    QFile file;
    file.setFileName(this->xmlWritePath);


    if (!file.open(QIODevice::ReadWrite | QIODevice::Truncate)){ // ouverture du fichier de sauvegarde
        std::cerr << "Error in opening file" << std::endl;
        file.close();
        return; // en écriture
    }
    std::cout << "file opened"<< std::endl;

    QTextStream textStream;
    textStream.setDevice(&file);
    textStream << this->doc->toString();
    file.close();
    delete this->doc;

}

void XMLManager::writeInterpolatedObjects(std::vector<RealObject *> objects)
{
    this->doc = new QDomDocument();
    QDomElement trackletsElement = this->doc->createElement("tracklets");
    this->doc->appendChild(trackletsElement);

    QDomElement countElement = this->doc->createElement("count");
    countElement.appendChild(this->doc->createTextNode(QString::number(objects.size())));
    trackletsElement.appendChild(countElement);

    QDomElement itemVersionElement = this->doc->createElement("item_version");
    itemVersionElement.appendChild(this->doc->createTextNode(QString::number(1)));
    trackletsElement.appendChild(itemVersionElement);

    for(int i = 0; i<objects.size(); i++){
        RealObject *robj = objects.at(i);

        //item (object)
        QDomElement itemElement = this->doc->createElement("item");
        //id(ajout perso)
        itemElement.setAttribute("id", robj->id);

        //objectType
        QDomElement objectTypeElement = this->doc->createElement("objectType");
        objectTypeElement.appendChild(this->doc->createTextNode(objectTypeToString(robj->objectType)));
        itemElement.appendChild(objectTypeElement);

        //h
        QDomElement hElement = this->doc->createElement("h");
        hElement.appendChild(this->doc->createTextNode(QString::number(robj->height)));
        itemElement.appendChild(hElement);
        //w
        QDomElement wElement = this->doc->createElement("w");
        wElement.appendChild(this->doc->createTextNode(QString::number(robj->width)));
        itemElement.appendChild(wElement);
        //l
        QDomElement lElement = this->doc->createElement("l");
        lElement.appendChild(this->doc->createTextNode(QString::number(robj->length)));
        itemElement.appendChild(lElement);

        //first_frame
        std::map<int, Pose3D>::iterator it = robj->annotations3DInterpolated.begin();
        int firstFrame = it->first;
        QDomElement firstFrameElement = this->doc->createElement("first_frame");
        firstFrameElement.appendChild(this->doc->createTextNode(QString::number(firstFrame)));
        itemElement.appendChild(firstFrameElement);

        //poses
        QDomElement posesElement = this->doc->createElement("poses");
        itemElement.appendChild(posesElement);



        //count (poses)

        //item_version (poses)

        for(it = robj->annotations3DInterpolated.begin(); it != robj->annotations3DInterpolated.end(); ++it){
            Pose3D pose = it->second;
            int currentFrame = it->first;

            //item
            QDomElement posesItemElement = this->doc->createElement("item");
            posesElement.appendChild(posesItemElement);

            QDomElement frameElement = this->doc->createElement("frame");
            frameElement.appendChild(this->doc->createTextNode(QString::number(currentFrame)));
            posesItemElement.appendChild(frameElement);

            //tx
            QDomElement txElement = this->doc->createElement("tx");
            txElement.appendChild(this->doc->createTextNode(QString::number(pose.x)));
            posesItemElement.appendChild(txElement);
            //ty
            QDomElement tyElement = this->doc->createElement("ty");
            tyElement.appendChild(this->doc->createTextNode(QString::number(pose.y)));
            posesItemElement.appendChild(tyElement);
            //tz
            QDomElement tzElement = this->doc->createElement("tz");
            tzElement.appendChild(this->doc->createTextNode(QString::number(pose.z)));
            posesItemElement.appendChild(tzElement);

            //ROTATIONS RANDOM !!!!!!!!!!!!!
            //rx
            QDomElement rxElement = this->doc->createElement("rx");
            rxElement.appendChild(this->doc->createTextNode(QString::number(pose.phi)));
            posesItemElement.appendChild(rxElement);
            //ry
            QDomElement ryElement = this->doc->createElement("ry");
            ryElement.appendChild(this->doc->createTextNode(QString::number(pose.psi)));
            posesItemElement.appendChild(ryElement);
            //rz
            QDomElement rzElement = this->doc->createElement("rz");
            rzElement.appendChild(this->doc->createTextNode(QString::number(pose.theta)));
            posesItemElement.appendChild(rzElement);

            //OCCULTATIONS A FAIRE !!!!!!!!!!!!!!!

        }

        trackletsElement.appendChild(itemElement);
    }

    QFile file;
    file.setFileName(this->xmlInterpolatedWritePath);
    std::cout << "(XMLManager::writeInterpolatedObjects) path : " << this->xmlInterpolatedWritePath.toStdString() << std::endl;


    if (!file.open(QIODevice::ReadWrite | QIODevice::Truncate)){ // ouverture du fichier de sauvegarde
        std::cerr << "Error in opening file" << std::endl;
        file.close();
        return; // en écriture
    }
    std::cout << "file opened"<< std::endl;

    QTextStream textStream;
    textStream.setDevice(&file);
    textStream << this->doc->toString();
    file.close();
    delete this->doc;
}

std::vector<RealObject *> XMLManager::readClickedObjects()
{

    std::vector<RealObject *> robjects;
    QDomDocument xmlBOM;
    // Load xml file as raw data
    QFile f(this->xmlReadPath);
    if (!f.open(QIODevice::ReadOnly ))
    {
        // Error while loading file
        std::cerr << "Error while loading file" << std::endl;
        return robjects;
    }
    // Set data into the QDomDocument before processing
    xmlBOM.setContent(&f);
    f.close();

    // Extract the root markup
    QDomElement root=xmlBOM.documentElement();

    // Get the first child of the root (Markup COMPONENT is expected)
    QDomElement Component=root.firstChild().toElement();

    // Loop while there is a child
    while(!Component.isNull())
    {
        // Check if the child tag name is COMPONENT
        if (Component.tagName()=="item")
        {
            RealObject *robj = new RealObject;
            int idRobj = Component.attribute("id").toInt();
            robj->id = idRobj;

            // Get the first child of the component
            QDomElement Child=Component.firstChild().toElement();

            // Read each child of the component node
            while (!Child.isNull())
            {
                // Read Name and value
                if (Child.tagName()=="objectType"){
                    //QString objectTypeStr =  Child.firstChild().toText().data();
                    QString objectTypeStr =  Child.text();
                    robj->objectType = stringToObject(objectTypeStr);
                }else if(Child.tagName()=="h"){
                    float hStr = Child.text().toFloat();
                    robj->height= hStr;
                }
                else if(Child.tagName()=="w"){
                    float hStr = Child.text().toFloat();
                    robj->width= hStr;
                }
                else if(Child.tagName()=="l"){
                    float hStr = Child.text().toFloat();
                    robj->length= hStr;
                }else if(Child.tagName()=="poses"){
                    QDomElement PoseItem = Child.firstChild().toElement();
                    while (!PoseItem.isNull()){
                        std::cout << "PoseItem tagName : " << PoseItem.tagName().toStdString() << std::endl;
                        QDomElement PoseComponent = PoseItem.firstChild().toElement();
                        Pose3D pose;
                        int frame = -1;
                        while (!PoseComponent.isNull()){
                            std::cout << "PoseComponent tagName : " << PoseComponent.tagName().toStdString() << std::endl;
                            if(PoseComponent.tagName()=="tx"){
                                float hStr = PoseComponent.text().toFloat();
                                pose.x = hStr;
                            }else if(PoseComponent.tagName()=="ty"){
                                float hStr = PoseComponent.text().toFloat();
                                pose.y= hStr;

                            }else if(PoseComponent.tagName()=="tz"){
                                float hStr = PoseComponent.text().toFloat();
                                pose.z= hStr;
                            }else if(PoseComponent.tagName()=="rx"){
                                float hStr = PoseComponent.text().toFloat();
                                pose.phi= hStr;
                            }else if(PoseComponent.tagName()=="ry"){
                                float hStr = PoseComponent.text().toFloat();
                                pose.psi= hStr;
                            }else if(PoseComponent.tagName()=="rz"){
                                float hStr = PoseComponent.text().toFloat();
                                pose.theta= hStr;
                            }else if(PoseComponent.tagName()=="frame"){
                                int hStr = PoseComponent.text().toInt();
                                frame = hStr;
                            }

                            PoseComponent = PoseComponent.nextSibling().toElement();
                        }
                        if(frame>=0){
                            robj->annotations3D[frame] = pose;
                        }

                        PoseItem = PoseItem.nextSibling().toElement();
                    }
                }

                // Next child
                Child = Child.nextSibling().toElement();
            }

            robjects.push_back(robj);
        }

        // Next component
        Component = Component.nextSibling().toElement();
    }


    return robjects;
}
