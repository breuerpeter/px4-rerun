# Changelog

## 0.1.0 (2026-04-14)


### Features

* add files ([11345d1](https://github.com/breuerpeter/px4-rerun/commit/11345d1052193a618f3eb344356eb1109f18b1dd))
* add messages blueprint with text log and trajectory views ([6207215](https://github.com/breuerpeter/px4-rerun/commit/6207215a2e80107f0f3fa9de2281ee2277198c9f))
* add readme ([63b70ea](https://github.com/breuerpeter/px4-rerun/commit/63b70ea68c0edb82cdcd2cac7a39dccbfe161926))
* add Rerun blueprints for viewer layouts ([4de884e](https://github.com/breuerpeter/px4-rerun/commit/4de884e3456ac23dc855efd9e71047feefd653b4))
* flight path as static LineStrips3D ([bcc22cc](https://github.com/breuerpeter/px4-rerun/commit/bcc22cccbfa2bcb3145af6dd4358a60247ebe886))
* integrate terrain fetcher, add CI, simplify CMake options ([abadc69](https://github.com/breuerpeter/px4-rerun/commit/abadc69c22fe4a64986ce2af533561a3c9ee38e5))
* **loader:** (untested) add files ([8bbd86c](https://github.com/breuerpeter/px4-rerun/commit/8bbd86cafc1b47859e9486d607ffb91bcbad93e1))
* log mag sensor and mag cal data ([9fcaf8d](https://github.com/breuerpeter/px4-rerun/commit/9fcaf8d8e35e54aa59c101ed7942469c1688af09))
* log setpoint ([b620947](https://github.com/breuerpeter/px4-rerun/commit/b620947d1cc680c8f70c093ddbedb9578b8c8cc7))
* log terrain ([2cbccd8](https://github.com/breuerpeter/px4-rerun/commit/2cbccd8a442d64c610e8f3d12a8b9813273df386))
* log velocity ([a895dee](https://github.com/breuerpeter/px4-rerun/commit/a895dee8a3ab2de395acab5d5a978bc9d5172d15))
* vehicle label ([69f82c3](https://github.com/breuerpeter/px4-rerun/commit/69f82c39933a53bbc978448c46f3561069f955f5))


### Bug Fixes

* catch std::exception instead of bare catch(...) ([cc574ea](https://github.com/breuerpeter/px4-rerun/commit/cc574ea8ce3bcb586030a117cb00f2bc72aa6f7d))
* combine position and attitude into single Transform3D ([f0c447e](https://github.com/breuerpeter/px4-rerun/commit/f0c447e45f5c4c0bcb565d6966c3e44e5d026902))
* default to not log all scalars ([ef109c5](https://github.com/breuerpeter/px4-rerun/commit/ef109c5093fb01cd02b82bf5faabdbe26150b306))
* don't use left-handed frame ([f518c54](https://github.com/breuerpeter/px4-rerun/commit/f518c5453c8299d734d9dd4db3d686eb895d518a))
* double promotion compiler warning ([87202e2](https://github.com/breuerpeter/px4-rerun/commit/87202e296c8e8b54aa705633b17a986358c5620a))
* hardcode loader app_id to px4-rerun for blueprint compatibility ([8d8cf56](https://github.com/breuerpeter/px4-rerun/commit/8d8cf5637f24f5fa83872bfbcddbd6f33402c9bb))
* keep terrain in NAVD88 to match PX4 ref_alt (AMSL) ([a08c026](https://github.com/breuerpeter/px4-rerun/commit/a08c0262a35829dccfff2efe8350b3c5fcfc3db2))
* log attitude and position independently ([6d8f30f](https://github.com/breuerpeter/px4-rerun/commit/6d8f30f4947f3b2bccb2c1724c7c823aa2c3d559))
* pin Rerun C++ SDK version ([f88991a](https://github.com/breuerpeter/px4-rerun/commit/f88991a04ed3b98803ad1ff886f63146fcbbe63e))
* quaternion convention ([4201648](https://github.com/breuerpeter/px4-rerun/commit/42016484d93f5f7433ea48020269b5da23ee188c))
* reduce padding for quicker processing ([0cc38ec](https://github.com/breuerpeter/px4-rerun/commit/0cc38ece5b11ae1e0da8a6ade97203b31bd92fd0))
* suppress third-party warnings on terrain_fetcher.cpp for PX4 SITL build ([32676b9](https://github.com/breuerpeter/px4-rerun/commit/32676b93a96eb6e9ddfe1f491946f6d1866ad01f))
