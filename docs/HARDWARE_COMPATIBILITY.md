# Hardware compatibility: DJI Mini 2 through an iPhone

Status as of 2026-11-26: **desk research only. Nothing on this page has been tested on
hardware.** No SDK has been downloaded, no sample has been built, no aircraft has been
connected. Every row below says so explicitly.

Target setup: original DJI Mini 2 (not Mini 2 SE, not Mini 4K), RC-N1 remote controller,
iPhone, MacBook for the build. Goal: read-only video, telemetry and connection state.
No flight control.

## Summary

- The only DJI SDK that covers the original Mini 2 on iOS is **Mobile SDK V4** (last
  version 4.16.2, October 2022). It is frozen. Mobile SDK V5 has no iOS version.
- On paper the read-only goal is feasible: Mini 2 support was added in V4.16, video comes
  through `DJIVideoFeeder`, telemetry through `DJIFlightControllerState`.
- It can only be built for a **physical iPhone**. The binary framework has no Apple
  Silicon simulator slice.
- The three things that can actually stop it are unknown until someone tries: whether
  4.16.2 builds and runs with the current Xcode and iOS, whether DJI's activation server
  still registers V4 app keys, and how the RC-N1 enumerates on a USB-C iPhone.

## Findings

Confidence is about the source, not about the result on your hardware.

| # | Finding | Source | Source date | Applies to | Limits | Confidence | Tested here |
|---|---------|--------|-------------|------------|--------|------------|-------------|
| 1 | Latest MSDK iOS V4 is 4.16.2; no later release | https://cocoapods.org/pods/DJI-SDK-iOS | Oct 2022 (inferred from the "last release" field) | SDK 4.16.2 | Frozen, no fixes expected | High | No |
| 2 | Distributed as CocoaPods `DJI-SDK-iOS` and via https://github.com/dji-sdk/Mobile-SDK-iOS . No Swift Package, no xcframework | same | Oct 2022 | SDK 4.16.2 | Plain `.framework` | High | No |
| 3 | CocoaPods trunk went into maintenance in Aug 2024 and becomes permanently read-only on 2 Dec 2026. Existing pods stay installable | https://blog.cocoapods.org/CocoaPods-Support-Plans/ | Aug 2024 | dependency tooling | Read-only date is days away: resolve and commit `Podfile.lock` early | Medium | No |
| 4 | Mini 2 (also Mini SE, Air 2S) supported from V4.16 on iOS | https://dronedj.com/2022/03/13/dji-ios-mobile-sdk-mini-2-air-2s-drone/ (press, not DJI) | Mar 2022 | SDK >= 4.16, Mini 2 | Required aircraft/RC firmware not found in any primary source | High for the claim, secondary source | No |
| 5 | MSDK V5 is Android-only; no iOS version planned | DJI SDK forum (login required, read from a search snippet); https://forum.flylitchi.com/t/mini2-support-for-ios/1991 (community) | 2022 | SDK V5 | Community / snippet evidence | High | No |
| 6 | `DJISDK.framework` has no arm64-simulator slice. On Apple Silicon the simulator build needs `EXCLUDED_ARCHS[sdk=iphonesimulator*] = arm64`; device builds are unaffected | https://forum.dji.com/thread-228303-1-1.html (community) | 2020-2022 | SDK 4.x, Apple Silicon | Device-only testing | Medium | No |
| 7 | App key registration goes through developer.dji.com; first launch needs a network call to DJI to activate, later launches use a local cache | https://developer.dji.com/api-reference/ios-api/Components/SDKManager/DJISDKManager.html | unknown | SDK 4.16.2 | Depends on DJI keeping the V4 activation service running | Medium | No |
| 8 | Info.plist needs `UISupportedExternalAccessoryProtocols` = `com.dji.video`, `com.dji.protocol`, `com.dji.common`, plus `DJISDKAppKey` | https://developer.dji.com/mobile-sdk/documentation/application-development-workflow/workflow-integrate.html | unknown | SDK 4.x | Bluetooth and local-network usage strings are not covered by DJI's page | High | No |
| 9 | Video: `DJIVideoFeeder` primary feed delivers H.264 to a `DJIVideoFeedListener`; `DJIWidget` / `DJIVideoPreviewer` decodes it | https://github.com/dji-sdk/DJIWidget | 2022 | SDK 4.16.2 | No Mini 2-specific issue documented, none ruled out | Medium | No |
| 10 | Telemetry: `DJIFlightControllerState` (attitude, GPS position, velocity, altitude above home), battery state, gimbal state | https://developer.dji.com/api-reference/ios-api/ | unknown | SDK 4.16.2 | Which keys the Mini 2 actually fills is untested | High for the API, unknown for the aircraft | No |
| 11 | Mini 2 under V4: no waypoint missions, no SDK flight simulator; virtual stick reported unreliable | DJI SDK forum, https://github.com/dji-sdk/Mobile-SDK-Android/issues/1282 (community) | 2022 | Mini 2 | Does not affect read-only use | Medium, community | No |
| 12 | Canada: under 250 g is a microdrone. No registration or pilot certificate; the general safety rules still apply | https://tc.canada.ca/en/aviation/drone-safety/learn-rules-you-fly-your-drone/drone-operation-categories-pilot-certificates/microdrones | current page | Mini 2 (249 g) | Check the page again before flying; a payload or prop guards can push the weight over 250 g | High | n/a |

Could not be found or verified:

- DJI's official supported-products matrix for V4 (the page returned 404).
- Any confirmed build of 4.16.2 with Xcode 16 or later, successful or failed.
- Any statement about `NSLocalNetworkUsageDescription` or Bluetooth usage strings.
- Minimum Mini 2 / RC-N1 firmware for V4.16.2.

## Risks, most likely first

1. **Toolchain age.** The SDK is four years older than the current Xcode. Expect at least
   deployment-target and linker warnings; a hard failure is possible and was neither
   confirmed nor excluded. First fixes to try: raise `IPHONEOS_DEPLOYMENT_TARGET` in the
   Podfile `post_install` hook, exclude the arm64 simulator slice.
2. **Activation service.** If DJI has retired V4 app-key activation the SDK will not
   register and there is no supported workaround. Test this before anything else is built.
3. **Cable and accessory protocol on USB-C iPhones.** The SDK talks to the remote through
   Apple's External Accessory framework. Whether an RC-N1 is exposed that way over
   USB-C-to-USB-C on a current iPhone is not documented in anything found. DJI's own Fly
   app working with the same cable is a good sign but not proof for third-party apps.
4. **CocoaPods read-only date (2 Dec 2026).** Installing existing pods should keep
   working; do the first `pod install` and commit `Podfile.lock` now anyway.
5. **Distribution.** Development builds only need the three protocol strings. App Store
   distribution of External Accessory apps involves MFi review and is out of scope.

If step 2 or 3 fails with no supported fix, the documented fallback is an Android device
with MSDK V4 (same aircraft support). Undocumented protocols and firmware modification are
not an option for this project.

## Feasibility test (Stage B and C of the hardware plan)

Needs: the MacBook with current Xcode, the iPhone, an Apple ID for signing, a DJI developer
account, the Mini 2 and RC-N1, **propellers removed**.

What to write down at each step is in the right column; send that back and the result gets
recorded here.

| Step | Do | Expected | Record |
|------|----|----------|--------|
| 1 | `sw_vers`, `uname -m`, `xcodebuild -version`, `pod --version`; iPhone model and iOS version | versions print | all five values |
| 2 | Register on developer.dji.com, create an app for MSDK **V4**, iOS, with your bundle id | an App Key is issued | whether V4 keys can still be created |
| 3 | `git clone https://github.com/dji-sdk/Mobile-SDK-iOS.git` then `pod install` in `Sample Code/SwiftSampleCode` | DJI-SDK-iOS 4.16.2 resolves | full output if it fails |
| 4 | Open the `.xcworkspace`, set bundle id, signing team and `DJISDKAppKey` in Info.plist | project opens | nothing |
| 5 | Build for the **iPhone** (not a simulator) | build succeeds | first error text if it fails |
| 6 | Run, with the phone online | registration callback reports success | the error code if it fails |
| 7 | Power the RC-N1 and Mini 2, connect RC-N1 to the phone | product-connected callback, model reads Mavic Mini 2 / Mini 2 | cable used, model string |
| 8 | Open the sample's camera / FPV screen | live video | resolution, visible delay |
| 9 | Open the flight-controller state screen | attitude, altitude, satellite count update | which fields stay empty |
| 10 | Unplug the cable, plug it back in | disconnect and reconnect are reported | time to recover |

Keep the app key, signing team id and any provisioning files out of Git.

## What happens after a successful test

Only then is `apps/ios-bridge/` worth building (see `docs/ROADMAP.md`, milestone v0.4.0):
a small Swift app that forwards the H.264 feed and a versioned telemetry message to the
desktop engine over the local network. The desktop side already has the pieces it will
plug into: `IFrameSource` (`input.type: stream`) and `Telemetry` with validity and a
`simulated` flag. Neither end sends anything to the aircraft.
