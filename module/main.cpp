#include <iostream>
#include <memory>
#include <sstream>

#include <boost/log/trivial.hpp>
#include <grpcpp/grpcpp.h>
#include <grpcpp/server_context.h>

#include <viam/sdk/common/exception.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/rpc/dial.hpp>
#include <viam/sdk/rpc/server.hpp>
#include <viam/sdk/services/service.hpp>
#include <viam/sdk/components/sensor.hpp>
#include <viam/sdk/components/camera.hpp>

#include <MagickWand/MagickWand.h>
#include <dmtx.h>

extern "C"
{
#include "../dmtxread/dmtxread.h"
}

using namespace viam::sdk;

class MatrixCodeReader : public Sensor, public Reconfigurable
{
public:
    void reconfigure(const Dependencies &deps, const ResourceConfig &cfg) override
    {
        std::cout << "MatrixCodeReader " << Resource::name() << " is reconfiguring" << std::endl;

        auto camera = cfg.attributes().find("camera");
        if (camera == cfg.attributes().end())
        {
            std::ostringstream buffer;
            buffer << Resource::name() << ": Required parameter `camera` not found in configuration";
            throw std::invalid_argument(buffer.str());
        }
        const ProtoValue &camera_val = camera->second;

        if (!camera_val.is_a<std::string>() || camera_val.get_unchecked<std::string>().empty())
        {
            throw std::invalid_argument("Invalid camera name, must be non-empty string");
        }
        std::string camera_name = camera_val.get_unchecked<std::string>();

        for (const auto &kv : deps)
        {
            if (kv.first.short_name() == camera_name)
            {
                camera_ = std::dynamic_pointer_cast<Camera>(kv.second);
            }
        }

        if (!camera_)
        {
            throw std::runtime_error("Camera dependency must be provided");
        }

        // TODO: have some configuration. use it for the implicit camera dependency.
    }

    ProtoStruct get_readings(const ProtoStruct &extra) override
    {
        std::cout << "MatrixCodeReader " << Resource::name() << " is getting readings" << std::endl;

        if (!camera_)
        {
            throw std::runtime_error("No camera found");
        }

        // TODO: error handling?
        auto image = camera_->get_image("image/jpeg");
        auto codes = parse_image(image.bytes);

        std::vector<ProtoValue> code_strings(codes.begin(), codes.end());

        ProtoStruct map;
        map.emplace("size", ProtoValue((int)image.bytes.size()));
        map.emplace("codes", code_strings);

        return map;
    }

    std::vector<std::string> parse_image(const std::vector<unsigned char> &image_bytes)
    {
        MagickWand *wand = NewMagickWand();

        MagickBooleanType success;

        success = MagickReadImageBlob(wand, image_bytes.data(), image_bytes.size());
        if (success == MagickFalse)
        {
            throw std::runtime_error("Failed to read image data");
        }

        UserOptions opt;
        DmtxImage *img;
        DmtxDecode *dec;
        DmtxRegion *reg;
        DmtxMessage *msg;
        int width, height;
        unsigned char *pxl;

        width = MagickGetImageWidth(wand);
        height = MagickGetImageHeight(wand);

        std::vector<std::string> codes;

        /* Loop once for each page within image */
        MagickResetIterator(wand);
        for (int imgPageIndex = 0; MagickNextImage(wand) != MagickFalse; imgPageIndex++)
        {
            /* Allocate memory for pixel data */
            pxl = (unsigned char *)malloc(3 * width * height * sizeof(unsigned char));
            if (pxl == NULL)
            {
                CleanupMagick(&wand, DmtxFalse);
                throw std::runtime_error("Failed to allocate memory for pixel data");
            }

            /* Copy pixels to known format */
            success = MagickGetImagePixels(wand, 0, 0, width, height, "RGB", CharPixel, pxl);
            if (success == MagickFalse || pxl == NULL)
            {
                CleanupMagick(&wand, DmtxTrue);
                throw std::runtime_error("Failed to make magick image pixels");
            }

            /* Initialize libdmtx image */
            img = dmtxImageCreate(pxl, width, height, DmtxPack24bppRGB);
            if (img == NULL)
            {
                CleanupMagick(&wand, DmtxFalse);
                throw std::runtime_error("Failed to create dmtx image");
            }

            dmtxImageSetProp(img, DmtxPropImageFlip, DmtxFlipNone);

            /* Initialize scan */
            dec = dmtxDecodeCreate(img, opt.shrinkMin);
            if (dec == NULL)
            {
                CleanupMagick(&wand, DmtxFalse);
                throw std::runtime_error("Failed to create dmtx decode");
            }

            // see if we have to set these options
            /*
            int err = SetDecodeOptions(dec, img, &opt);
            if (err != DmtxPass)
            {
                CleanupMagick(&wand, DmtxFalse);
                throw std::runtime_error("Failed to set dmtx decode options");
            }
            */

            /* Find and decode every barcode on page */
            int pageScanCount = 0;
            int imgScanCount = 0;
            for (;;)
            {
                /* Find next barcode region within image, but do not decode yet */
                reg = dmtxRegionFindNext(dec, NULL);

                /* Finished file or ran out of time before finding another region */
                if (reg == NULL)
                    break;

                msg = dmtxDecodeMatrixRegion_new(dec, reg, opt.correctionsMax, &opt);

                if (msg != NULL)
                {
                    std::string hex_str;
                    for (size_t i = 0; i < msg->arraySize; i++)
                    {
                        char hex[3];
                        snprintf(hex, sizeof(hex), "%02X", msg->array[i]);
                        hex_str += hex;
                        hex_str += " ";
                    }
                    codes.push_back(hex_str);

                    // PrintStats(dec, reg, msg, imgPageIndex, &opt);
                    // PrintMessage(reg, msg, &opt);

                    pageScanCount++;
                    imgScanCount++;

                    dmtxMessageDestroy(&msg);
                }

                dmtxRegionDestroy(&reg);

                if (opt.stopAfter != DmtxUndefined && imgScanCount >= opt.stopAfter)
                    break;
            }

            dmtxDecodeDestroy(&dec);
            dmtxImageDestroy(&img);
            free(pxl);
        }

        CleanupMagick(&wand, DmtxFalse);

        return codes;
    }

    ProtoStruct
    do_command(const ProtoStruct &command) override
    {
        throw std::runtime_error("Not implemented");
    }

    std::vector<GeometryConfig> get_geometries(const ProtoStruct &extra) override
    {
        throw std::runtime_error("Not implemented");
    }

    MatrixCodeReader(Dependencies deps, ResourceConfig cfg) : Sensor(cfg.name())
    {
        std::cout << "Creating MatrixCodeReader " + Resource::name() << std::endl;
        MagickWandGenesis();
        reconfigure(deps, cfg);
    }

    ~MatrixCodeReader()
    {
        MagickWandTerminus();
    }

private:
    std::shared_ptr<Camera> camera_;
};

int main(int argc, char **argv)
try
{
    API generic = API::get<Sensor>();
    Model m("hpe", "sensor", "matrix_code");

    std::shared_ptr<ModelRegistration> mr = std::make_shared<ModelRegistration>(
        generic,
        m,
        [](Dependencies deps, ResourceConfig cfg)
        { return std::make_unique<MatrixCodeReader>(deps, cfg); });

    std::vector<std::shared_ptr<ModelRegistration>> mrs = {mr};
    auto my_mod = std::make_shared<ModuleService>(argc, argv, mrs);
    my_mod->serve();

    return EXIT_SUCCESS;
}
catch (const viam::sdk::Exception &ex)
{
    std::cerr << "main failed with exception: " << ex.what() << "\n";
    return EXIT_FAILURE;
}