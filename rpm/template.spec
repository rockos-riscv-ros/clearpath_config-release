%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-clearpath-config
Version:        0.3.3
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS clearpath_config package

License:        BSD-3
Source0:        %{name}-%{version}.tar.gz

Requires:       python%{python3_pkgversion}-yaml
Requires:       ros-humble-ros-workspace
BuildRequires:  python%{python3_pkgversion}-devel
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  ros-humble-ament-copyright
BuildRequires:  ros-humble-ament-flake8
BuildRequires:  ros-humble-ament-pep257
%endif

%description
Clearpath Configuration YAML Parser and Writer

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/humble"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Sun Sep 29 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.3.3-1
- Autogenerated by Bloom

* Mon Sep 23 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.3.2-1
- Autogenerated by Bloom

* Thu Sep 19 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.3.1-1
- Autogenerated by Bloom

* Thu Sep 19 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.3.0-1
- Autogenerated by Bloom

* Thu Aug 08 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.11-1
- Autogenerated by Bloom

* Mon Jul 22 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.10-1
- Autogenerated by Bloom

* Tue May 28 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.9-1
- Autogenerated by Bloom

* Tue May 14 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.8-1
- Autogenerated by Bloom

* Mon Apr 08 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.7-1
- Autogenerated by Bloom

* Wed Mar 06 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.5-1
- Autogenerated by Bloom

* Mon Jan 22 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.4-1
- Autogenerated by Bloom

* Wed Jan 10 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.3-1
- Autogenerated by Bloom

* Mon Jan 08 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.2-1
- Autogenerated by Bloom

* Thu Jan 04 2024 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.1-1
- Autogenerated by Bloom

* Thu Dec 07 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.2.0-2
- Autogenerated by Bloom

* Mon Oct 02 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.1.1-1
- Autogenerated by Bloom

* Thu Aug 31 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.1.0-1
- Autogenerated by Bloom

* Thu Aug 10 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.6-1
- Autogenerated by Bloom

* Mon Jul 31 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.5-1
- Autogenerated by Bloom

* Mon Jul 17 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.4-1
- Autogenerated by Bloom

* Fri Jul 07 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.3-3
- Autogenerated by Bloom

* Thu Jul 06 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.3-1
- Autogenerated by Bloom

* Tue Jun 20 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.2-2
- Autogenerated by Bloom

* Mon Jun 12 2023 Luis Camero <lcamero@clearpathrobotics.com> - 0.0.2-1
- Autogenerated by Bloom

