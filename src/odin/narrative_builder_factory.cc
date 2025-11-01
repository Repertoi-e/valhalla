#include "odin/narrative_builder_factory.h"
#include "odin/enhancedtrippath.h"
#include "odin/markup_formatter.h"
#include "odin/narrativebuilder.h"
#include "odin/util.h"
#include "proto/options.pb.h"

namespace valhalla {
namespace odin {

std::unique_ptr<NarrativeBuilder>
NarrativeBuilderFactory::Create(const Options& options,
                                const EnhancedTripLeg* trip_path,
                                const MarkupFormatter& markup_formatter) {

  const auto phrase_dictionary = get_locales_ensure_narrative_dictionary(options.language());
  if (!phrase_dictionary) {
    throw std::runtime_error("Invalid language tag.");
  }

  // if a NarrativeBuilder is derived with specific code for a particular
  // language then add logic here and return derived NarrativeBuilder
  if (phrase_dictionary->GetLanguageTag() == "cs-CZ") {
    return std::make_unique<NarrativeBuilder_csCZ>(options, trip_path, *phrase_dictionary,
                                                   markup_formatter);
  } else if (phrase_dictionary->GetLanguageTag() == "hi-IN") {
    return std::make_unique<NarrativeBuilder_hiIN>(options, trip_path, *phrase_dictionary,
                                                   markup_formatter);
  } else if (phrase_dictionary->GetLanguageTag() == "it-IT") {
    return std::make_unique<NarrativeBuilder_itIT>(options, trip_path, *phrase_dictionary,
                                                   markup_formatter);
  } else if (phrase_dictionary->GetLanguageTag() == "ru-RU") {
    return std::make_unique<NarrativeBuilder_ruRU>(options, trip_path, *phrase_dictionary,
                                                   markup_formatter);
  }

  // otherwise just return pointer to NarrativeBuilder
  return std::make_unique<NarrativeBuilder>(options, trip_path, *phrase_dictionary,
                                            markup_formatter);
}

} // namespace odin
} // namespace valhalla
